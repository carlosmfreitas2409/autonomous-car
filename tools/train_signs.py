"""Train the sign detector and export the ONNX the car actually runs.

ultralytics and torch are needed here and nowhere else -- the car loads the
exported ONNX through onnxruntime alone. Install with:

    pip install -r requirements-dev.txt

Bootstrap, before any custom sign exists. Exports the stock COCO yolov8n,
which covers stop signs and traffic lights well enough to exercise the whole
chain -- threading, tracking, light reading, braking, GUI -- on real hardware:

    python3 tools/train_signs.py --bootstrap

Real training, once tools/synthesize_dataset.py has produced a dataset:

    python3 tools/train_signs.py --data dataset/synth/data.yaml --epochs 80

Class names travel inside the exported ONNX metadata, so src/ never needs to
know what was trained.
"""

import argparse
import shutil
from pathlib import Path


DEFAULT_OUTPUT = Path("models/signs.onnx")
DEFAULT_IMGSZ = 640

# Stop once the score has not improved for this many epochs. Synthetic data
# plateaus early, so this usually ends training well short of --epochs.
DEFAULT_PATIENCE = 20


def pick_device(requested: str | None) -> str:
    """Choose the accelerator.

    Ultralytics' own auto-selection returns "cpu" on Apple silicon even when
    MPS is available and working, which is the difference between minutes per
    epoch and a quarter of an hour. So we choose explicitly.
    """
    if requested:
        return requested

    import torch

    if torch.cuda.is_available():
        return "0"
    if torch.backends.mps.is_available():
        return "mps"
    return "cpu"


def export(model, imgsz: int, output: Path) -> None:
    exported = model.export(format="onnx", imgsz=imgsz, opset=12)
    output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy(exported, output)
    size_mb = output.stat().st_size / 1e6
    print(f"\nWrote {output} ({size_mb:.1f} MB)")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bootstrap", action="store_true", help="export stock COCO weights instead of training")
    parser.add_argument("--data", type=Path, help="data.yaml from synthesize_dataset.py")
    parser.add_argument("--base", default="yolov8n.pt", help="starting weights")
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--imgsz", type=int, default=DEFAULT_IMGSZ)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--device", help="mps, cpu, or a CUDA index. Auto-detected by default.")
    parser.add_argument("--patience", type=int, default=DEFAULT_PATIENCE, help="stop after N epochs with no gain")
    parser.add_argument("--cache", action="store_true", help="hold decoded images in RAM between epochs")
    args = parser.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit("ultralytics is not installed. pip install -r requirements-dev.txt")

    if args.bootstrap:
        print(f"Exporting stock {args.base} (COCO classes) as a bootstrap model")
        export(YOLO(args.base), args.imgsz, args.output)
        print(
            "\nThis is COCO-pretrained, so only stop signs and traffic lights are\n"
            "detectable and accuracy on scale-model signs will be poor. It exists to\n"
            "prove the pipeline end to end, not to drive the track."
        )
        return

    if args.data is None:
        raise SystemExit("Pass --data dataset/synth/data.yaml, or --bootstrap")
    if not args.data.exists():
        raise SystemExit(f"No dataset at {args.data}. Run tools/synthesize_dataset.py first.")

    device = pick_device(args.device)
    print(f"Training on {device} at {args.imgsz}px, up to {args.epochs} epochs (patience {args.patience})")
    if device == "cpu":
        print("  No accelerator found. Expect this to take hours - consider --imgsz 480.")

    model = YOLO(args.base)
    model.train(
        data=str(args.data),
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        device=device,
        patience=args.patience,
        cache=args.cache,
        project="runs",
    )
    export(model, args.imgsz, args.output)
    print("\nNext: score it against real photos with tools/eval_signs.py")


if __name__ == "__main__":
    main()
