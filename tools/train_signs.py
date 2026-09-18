#!/usr/bin/env python3
"""Fine-tune YOLO on the synthetic set and export one ONNX file.

Runs on the training box (Ryzen 7 5700X, RTX 3060), never on the Mac: this is
the only file in the project that imports ultralytics, and ultralytics brings
torch, which is ~2.5 GB. The hand-off between the two machines is one file,
`models/signs.onnx`. The dataset, the weights and the run logs stay here and
are gitignored.

Train both sizes. On a 3060 that is one extra unattended run, so there is no
reason to guess which is better:

    python3 tools/train_signs.py --base yolov8n.pt --name signs_n
    python3 tools/train_signs.py --base yolov8s.pt --name signs_s

then copy both ONNX files to the Mac and let tools/eval_signs.py and
tools/probe_signs.py pick the winner on measured real-photo recall at
acceptable latency. A sign camera does not need 30 fps - it needs to not miss
the light.

`imgsz` is 640 and is not a parameter. At 320 the traffic light was missed
outright.
"""

import argparse
import shutil
from pathlib import Path

IMAGE_SIZE = 640
OPSET = 12


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
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
        # The compositor already varies scale, perspective, exposure, blur and
        # noise deliberately and per class. Letting ultralytics do it again on
        # top would undo the per-class paste sizes, which are the point.
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
