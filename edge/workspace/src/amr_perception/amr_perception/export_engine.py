import argparse
import os

from ultralytics import YOLOE


def main():
    parser = argparse.ArgumentParser(description='Export a YOLOE model to a TensorRT FP16 engine for object_detector.')
    parser.add_argument('model', help='e.g. yoloe-26s-seg-pf.pt, or yoloe-26s-seg.pt together with --classes')
    parser.add_argument('--classes', nargs='+', default=[], help='class names to bake into the engine')
    parser.add_argument('--output', help='engine file name (default: <model name>.engine)')
    parser.add_argument('--models-dir', default='/data/models')
    args = parser.parse_args()

    os.makedirs(args.models_dir, exist_ok=True)
    os.chdir(args.models_dir)
    model = YOLOE(args.model)
    if args.classes:
        model.set_classes(args.classes)
    # エンジンは書き出したGPUとTensorRTのバージョンでしか動かないため、それらが変わったら作り直す。
    engine = model.export(format='engine', half=True)
    if args.output:
        os.replace(engine, args.output)
        engine = args.output
    print(os.path.join(args.models_dir, engine))


if __name__ == '__main__':
    main()
