#!/usr/bin/env python3


import argparse
import shutil
from pathlib import Path

IMAGE_SIZE = 640
OPSET = 12


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", default="dataset/synth/data.yaml")
    parser.add_argument("--base", default="yolov8n.pt")
    parser.add_argument("--name", default="signs")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch", type=int, default=32)
    parser.add_argument("--patience", type=int, default=20, help="early stopping")
    parser.add_argument("--device", default="0", help="'0' for the 3060, 'cpu' to limp")
    parser.add_argument("--out", default="models")
    args = parser.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        print("ultralytics is not installed. This script runs on the training box:")
        print("  pip install -r requirements-dev.txt")
        return 2

    data = Path(args.data)
    if not data.exists():
        print(f"no dataset at {data} - run tools/synthesize_dataset.py first")
        return 2

    model = YOLO(args.base)
    model.train(
        data=str(data),
        imgsz=IMAGE_SIZE,
        epochs=args.epochs,
        batch=args.batch,
        patience=args.patience,
        device=args.device,
        amp=True,
        name=args.name,
        project="runs",
        degrees=0.0, translate=0.05, scale=0.1, shear=0.0, perspective=0.0,
        mosaic=0.0, mixup=0.0, hsv_h=0.010, hsv_s=0.3, hsv_v=0.3,
    )

    exported = Path(model.export(format="onnx", imgsz=IMAGE_SIZE, opset=OPSET, simplify=True))
    destination = Path(args.out) / f"{args.name}.onnx"
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy(exported, destination)

    print(f"\n{destination}")
    print("Copy that one file to the Mac, then there:")
    print(f"  python3 tools/probe_signs.py --model {destination}")
    print(f"  python3 tools/eval_signs.py --model {destination} --dataset assets/real")
    print("  python3 tools/eval_light_state.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
