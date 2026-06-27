''' 
Offline latency benchmark for emotion recognition ONNX models deployed
on the Jetson Orin Nano. Measures steady state latency and preprocessing
cost for each model found under src/x3_visual/models/emotion_recognition

Usage: 
    python benchmark_emotion_recognition.py [--model_dir PATH]
                                            [--provider PROVIDER]
                                            [--n_warmup N]
                                            [--n_runs N]
                                            [--output_dir PATH]
    --models_dir    Path to the emotion_recognition/ folder containing
                    subdirectories for each model (with model.onnx and 
                    onnx_config.json)
                    Default: src/x3_visual/models/ directory
    --provider      Execution provider: "trt", "cuda", or "cpu". 
                    Default: "trt", provided the TRT engines were compiled
    --n_warmup      Number of warmup passes to discard. Default: 50.
    --n_runs        Number of timed passes per model. Default: 500.
    --output_dir    Directory to write the benchmark_results.csv and latency plots
                    Defaults: ./benchmark_outputs
'''

import os, json, time
import argparse
import numpy as np
import cv2
import onnxruntime as ort
import csv
import matplotlib.pyplot as plt
from pathlib import Path
from dataclasses import dataclass, field, asdict
from typing import Optional

# ========== Resulting dataclass ==========
@dataclass
class ModelBenchmarkResult:
    # Model metadata
    model_name:         str
    model_class:        str
    model_variant:      str
    dataset:            str
    input_hw:           list
    color_mode:         str
    provider:           str

    # Inference latency - for session.run()
    infer_mean_ms:      float
    infer_std_ms:       float
    infer_medians_ms:   float
    infer_p95_ms:       float
    infer_p99_ms:       float
    infer_min_ms:       float
    infer_max_ms:       float
    infer_fps:          float

    # End-to-end (preprocessing + inference)
    e2e_mean_ms:        float
    e2e_std_ms:         float
    e2e_p95_ms:         float
    e2e_p99_ms:         float
    e2e_min_ms:         float
    e2e_max_ms:         float
    e2e_fps:            float

    n_warmup:           int
    n_runs:             int

def load_session(model_path: str, provider: str, trt_cache_dir: str):
    if provider == "trt":
        providers = [
            ('TensorrtExecutionProvider', {
                'devide_id':                0,
                'trt_max_workspace_size':   512 * 1024 * 1024,
                'trt_fp16_enable':          True,
                'trt_engine_cache_enable':  True,
                'trt_engine_cache_path':    trt_cache_dir,
            }),
            ('CUDAExecutionProvider', {'device_id': 0}),
            'CPUExecutionProvider',
        ]
    elif provider == "cuda":
        providers = [
            ('CUDAExecutionProvider', {'device_id': 0}),
            'CPUExecutionProvider',
        ]
    else:
        providers = ['CPUExecutionProvider']

    session_options = ort.SessionOptions()
    session_options.log_severity_level = 3

    session = ort.InferenceSession(
        model_path,
        session_options=session_options,
        providers=providers
    )

    return session

def preprocess(cv_image: np.ndarray,
               input_h: int, input_w: int,
               color_mode: str,
               normalize_mean: Optional[list],
               normalize_std: Optional[list]) -> np.ndarray:
    '''
    Resize, color-convert, normalize, and reformat a BGR uint8 image into a 
    (1, 3, H, W) float32 tensor ready for ORT inference.

    Mirrors emotion_recognition_node._preprocess() method such that inference
    performance reflect the real preprocessing cost in the deployed pipeline
    '''
    img = cv2.resize(cv_image, (input_w, input_h))

    if color_mode == "grayscale":
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    else:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    img = img.astype(np.float) / 255.0

    if normalize_mean is not None:
        mean = np.array(normalize_mean, dtype=np.float32)
        std  = np.array(normalize_std,  dtype=np.float32)
        img  = (img - mean) / std

    img = img.transpose(2, 0, 1)        # (H, W, C) -> (C, H, W)
    img = np.ascontiguousarray(img)
    img = np.expand_dims(img, axis=0)   # -> (1, C, H, W)

    return img

def computer_stats(duration_s: np.ndarray) -> dict:
    '''
    Convert an array of per-run durations (in seconds) to a statistic dict
    with all values in milliseconds
    '''
    ms = duration_s * 1000.0
    return {
        'mean_ms':      float(np.mean(ms)),
        'std_ms':       float(np.std(ms)),
        'median_ms':    float(np.percentile(ms, 50)),
        'p95_ms':       float(np.percentile(ms, 95)),
        'p99_ms':       float(np.percentile(ms, 99)),
        'min_ms':       float(np.min(ms)),
        'max_ms':       float(np.max(ms)),   
    }

def benchmark_model(model_name: str,
                    model_dir: Path,
                    provider: str,
                    n_warmup: int,
                    n_runs: int):
    ''' 
    Run the full benchmark for a single emotion recognition model.
    
    Steps:
        1. Load the onnx_config.json,
        2. Load ORT session (triggers TRT enginer build/cache if provider="trt")
        3. Build a synthetic BGR face-crop image as the benchmark input
        4. Warm- passes, which are discarded
        5. Time preprocessing loop
        6. Timed inference loop
        7. Compute and return statistics    
    '''

    onnx_path   = model_dir / 'model.onnx'
    config_path = model_dir / 'onnx_config.json'

    if not onnx_path.exists():
        raise FileNotFoundError(f"model.onnx not found at {onnx_path}")
    if not config_path.exists():
        raise FileNotFoundError(f"onnx_config.json not found at {config_path}")
    
    # ========== 1. Load config ==========
    with open(config_path) as f:
        config = json.load(f)

    input_h, input_w    = config['input_hw']
    color_mode          = config['color_mode']
    normalize_mean      = config['normalize_mean']
    normalize_std       = config['normalize_std']
    model_class         = config.get('model_class', 'unknown')
    model_variant       = config.get('model_variant', 'unknown')
    dataset             = config.get('dataset', 'unknown')

    print(f"\n{'='*60}")
    print(f"  Model      : {model_name}")
    print(f"  Class      : {model_class} ({model_variant})")
    print(f"  Dataset    : {dataset}")
    print(f"  Input      : {input_h}x{input_w}  color_mode={color_mode}")
    print(f"  Provider   : {provider}")
    print(f"  Warm-up    : {n_warmup}  |  Timed runs: {n_runs}")
    print(f"{'='*60}")

    # ========== 2. Load ORT session ==========
    print("  Loading ORT session...", end=' ', flush=True)
    session = load_session(
        model_path=str(onnx_path),
        provider=provider,
        trt_cache_dir=str(model_dir),
    )

    input_name  = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name
    print(f"done  [{session.get_providers()[0]}]")

    rng = np.random.default_rng(seed=42)

    # ========== 3. Warm-up passes ==========
    # Prime CUDA kernels, TRT execution contexts, and memory allocations
    # to amortize and prevent inflating timed measurements
    print(f"  Warming up ({n_warmup} passes)...", end=' ', flush=True)
    for _ in range(n_warmup):
        crop = rng.integers(0, 256, (input_h, input_w, 3), dtype=np.uint8)
        tensor = preprocess(crop, input_h, input_w,
                            color_mode, normalize_mean, normalize_std)
        session.run([output_name], {input_name: tensor})
    print("done")

    # ========== 4. Timed loops ==========
    print(f"  Measuring processing + inference time ({n_runs} passes)... ", end=' ', flush=True)
    infer_durations = np.empty(n_runs, dtype=np.float64)
    e2e_durations   = np.empty(n_runs, dtype=np.float64)
    for i in range(n_runs):
        crop = rng.integers(0, 256, (input_h, input_w, 3), dtype=np.uint8)
        t0 = time.perf_counter()
        tensor = preprocess(crop, input_h, input_w, color_mode, normalize_mean, normalize_std)
        t1 = time.perf_counter()
        session.run([output_name], {input_name: tensor})
        t2 = time.perf_counter()
        infer_durations[i]  = t2 - t1
        e2e_durations[i]    = t2 - t0
    print("done")

    # ========== 5. Compute statistics ==========
    infer_stats = computer_stats(infer_durations)
    e2e_stats   = computer_stats(e2e_durations)

    print(f"\n  --- Inference (session.run only) ---")
    print(f"  mean={infer_stats['mean_ms']:.2f}ms  std={infer_stats['std_ms']:.2f}ms  "
          f"median={infer_stats['median_ms']:.2f}ms  "
          f"P95={infer_stats['p95_ms']:.2f}ms  "
          f"P99={infer_stats['p99_ms']:.2f}ms  "
          f"FPS={1000/infer_stats['mean_ms']:.1f}")
    print(f"\n  --- End-to-End (preprocess + inference) ---")
    print(f"  mean={e2e_stats['mean_ms']:.2f}ms  std={e2e_stats['std_ms']:.2f}ms  "
          f"median={e2e_stats['median_ms']:.2f}ms  "
          f"P95={e2e_stats['p95_ms']:.2f}ms  "
          f"P99={e2e_stats['p99_ms']:.2f}ms  "
          f"FPS={1000/e2e_stats['mean_ms']:.1f}")
    
    # --- Clean up session to free GPU memory before the next model
    del session
 
    result = ModelBenchmarkResult(
        model_name      = model_name,
        model_class     = model_class,
        model_variant   = str(model_variant),
        dataset         = dataset,
        input_hw        = [input_h, input_w],
        color_mode      = color_mode,
        provider        = provider,
 
        infer_mean_ms   = infer_stats['mean_ms'],
        infer_std_ms    = infer_stats['std_ms'],
        infer_median_ms = infer_stats['median_ms'],
        infer_p95_ms    = infer_stats['p95_ms'],
        infer_p99_ms    = infer_stats['p99_ms'],
        infer_min_ms    = infer_stats['min_ms'],
        infer_max_ms    = infer_stats['max_ms'],
        infer_fps       = 1000.0 / infer_stats['mean_ms'],
 
        e2e_mean_ms     = e2e_stats['mean_ms'],
        e2e_std_ms      = e2e_stats['std_ms'],
        e2e_median_ms   = e2e_stats['median_ms'],
        e2e_p95_ms      = e2e_stats['p95_ms'],
        e2e_p99_ms      = e2e_stats['p99_ms'],
        e2e_min_ms      = e2e_stats['min_ms'],
        e2e_max_ms      = e2e_stats['max_ms'],
        e2e_fps         = 1000.0 / e2e_stats['mean_ms'],
 
        n_warmup        = n_warmup,
        n_runs          = n_runs,
    )
 
    # Return raw arrays in ms for plotting
    return result, infer_durations * 1000.0, e2e_durations * 1000.0

def plot_latency_histogram(infer_duration_ms: np.ndarray,
                           e2e_duration_ms: np.ndarray,
                           model_name: str,
                           output_path: Path):
    '''
    Two-panel histogram: inference-only and e2e latency
    Vertical lines mark mean, P95, and P99 on each panel
    '''
    fig, axes = plt.subplots(1, 2, figsize=(14,4))
    fig.suptitle(f'Latency distribution - {model_name}', fontsize=13)

    for ax, durations, title in zip(
        axes,
        [infer_duration_ms, e2e_duration_ms],
        ['Inference', 'End-to-End'],
    ):
        ax.hist(durations, bins=50, color='steelblue', edgecolor='white', linewidth=0.3)
        ax.axvline(np.mean(durations),
                   color='tomato',     linestyle='--', linewidth=1.5,
                   label=f'Mean  {np.mean(durations):.2f} ms')
        ax.axvline(np.percentile(durations, 95),
                   color='darkorange', linestyle=':',  linewidth=1.5,
                   label=f'P95   {np.percentile(durations, 95):.2f} ms')
        ax.axvline(np.percentile(durations, 99),
                   color='crimson',    linestyle='-.', linewidth=1.5,
                   label=f'P99   {np.percentile(durations, 99):.2f} ms')
        ax.set_xlabel('Latency (ms)')
        ax.set_ylabel('Count')
        ax.set_title(title)
        ax.legend(fontsize=9)
 
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Plot saved: {output_path.name}")

# ---------------------------------------------------------------------------
# CSV + JSON output
# ---------------------------------------------------------------------------
 
CSV_FIELDS = [
    'model_name', 'model_class', 'model_variant', 'dataset',
    'input_hw', 'color_mode', 'provider',
    'infer_mean_ms', 'infer_std_ms', 'infer_median_ms',
    'infer_p95_ms', 'infer_p99_ms', 'infer_min_ms', 'infer_max_ms', 'infer_fps',
    'e2e_mean_ms', 'e2e_std_ms', 'e2e_median_ms',
    'e2e_p95_ms', 'e2e_p99_ms', 'e2e_min_ms', 'e2e_max_ms', 'e2e_fps',
    'n_warmup', 'n_runs',
]
 
def save_csv(results: list[ModelBenchmarkResult], output_path: Path):
    with open(output_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for r in results:
            row = asdict(r)
            row['input_hw'] = f"{r.input_hw[0]}x{r.input_hw[1]}"
            writer.writerow({k: row[k] for k in CSV_FIELDS})
    print(f"\nCSV saved: {output_path}")
 
 
def save_json(results: list[ModelBenchmarkResult], output_path: Path):
    data = []
    for r in results:
        d = asdict(r)
        data.append(d)
    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"JSON saved: {output_path}")

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description='Offline latency benchmark for emotion recognition ONNX models'
    )
    parser.add_argument(
        '--models_dir', type=str, required=True,
        help='Path to the emotion_recognition/directory (contains per-model subdirs)'
    )
    parser.add_argument(
        '---provider', type=str, default='cuda', choices=['trt', 'cuda', 'cpu'],
        help='ORT execution provider. Use "trt" on Jetson after engine compilation. Default: cuda'
    )
    parser.add_argument(
        '---n_warmup', type=int, default=50,
        help='Number of warm-up passes to discard before timing. Default: 50'
    )
    parser.add_argument(
        '---n_runs', type=int, default=500,
        help='Number of timed passes per model. Default: 500'
    )
    parser.add_argument(
        '--output_dir', type=str, default='./benchmark_output',
        help='Directory to write the results CSV, JSON, and plots. Default: ./benchmark_output'
    )
    parser.add_argument(
        '--models', type=str, default=None,
        help='Benchmark a single named model directory instead of all models.'
             'Must be a subdirectory name within --models_dir'
    )
    args = parser.parse_args()

    models_dir = Path(args.models_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    if not models_dir.exists():
        raise FileNotFoundError(f"models_dir not found: {models_dir}")
    
    if args.model is not None:
        model_dirs = [models_dir / args.model]
    else:
        model_dirs = sorted([
            d for d in models_dir.iterdir() if d.is_dir()
            and (d / 'model.onnx').exists()
            and (d / 'onnx_config.json').exists()
        ])

    if not model_dirs:
        print(f"No valid model directories found under {models_dir}")
        print(f"Each model directory must contain model.onnx and onnx_config.json")
        return
    
    print(f"\nFound {len(model_dirs)} model(s) to benchmark:")
    for d in model_dirs:
        print(f"  {d.name}")

    # --- Run benchmark for each model
    # Models are benchmarked one at a time
    results = []
    failed = []
    raw_infer:  dict[str, np.ndarray] = {}
    raw_e2e:    dict[str, np.ndarray] = {}

    for model_dir in model_dirs:
        model_name = model_dir.name
        try:
            result, infer_ms, e2e_ms = benchmark_model(
                model_name  = model_name,
                model_dir   = model_dir,
                provider    = args.provider,
                n_warmup    = args.n_warmup,
                n_runs      = args.n_run,
            )
            results.append(result)
            raw_infer[model_name] = infer_ms
            raw_e2e[model_name]   = e2e_ms

        except Exception as e:
            print(f"\n  Error benchmarking {model_name}: {e}")
            failed.append((model_name, str(e)))
            continue
            
    if results:
        csv_path  = output_dir / 'benchmark_results.csv'
        json_path = output_dir / 'benchmark_summary.json'
        save_csv(results, csv_path)
        save_json(results, json_path)

        # --- Per-model latency histograms
        for r in results:
            plot_latency_histogram(
                infer_duration_ms = raw_infer[r.model_name],
                e2e_duration_ms   = raw_e2e[r.model_name],
                model_name        = r.model_name,
                output_path       = output_dir / f"{r.model_name}_latency.png"
            )

        # --- Print summary table
        print(f"\n{'='*100}")
        print(f"{'Model':<35} {'Provider':<6} {'Infer mean±std (ms)':<24} {'Infer P95':>10} {'Infer FPS':>10} {'E2E FPS':>8}")
        print(f"{'='*100}")
        for r in results:
            print(f"{r.model_name:<35} {r.provider:<6} "
                  f"{r.infer_mean_ms:>7.2f} ± {r.infer_std_ms:<10.2f}"
                  f"{r.infer_p95_ms:>10.2f} {r.infer_fps:>10.1f} {r.e2e_fps:>8.1f}")
        print(f"{'='*100}")

    if failed:
        print(f"\nFailed models ({len(failed)}):")
        for name, err in failed:
            print(f"  {name}: {err}")

    print(f"\nDone. Results written to: {output_dir.resolve()}")

if __name__ == "__main__":
    main()