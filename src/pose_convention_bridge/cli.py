import argparse
import sys
from .common import load, emit
from .core import read_poses, similarity, convert, output_document


def main(argv=None):
    parser=argparse.ArgumentParser(description='Explicit camera-pose direction and axis convention converter.')
    parser.add_argument('input')
    parser.add_argument('--input-format',choices=['json','tum','colmap-images'],default='json')
    parser.add_argument('--input-direction',choices=['c2w','w2c'],required=True)
    parser.add_argument('--input-axes',choices=['opencv','opengl'],required=True)
    parser.add_argument('--output-direction',choices=['c2w','w2c'],default='c2w')
    parser.add_argument('--output-axes',choices=['opencv','opengl'],default='opengl')
    parser.add_argument('--format',choices=['generic','nerfstudio'],default='generic')
    group=parser.add_mutually_exclusive_group()
    group.add_argument('--world-similarity',help='JSON {matrix:rigid4x4,scale:s}; p_out=s*R*p_in+t.')
    group.add_argument('--dataparser-transform',help='Explicit Nerfstudio {transform:3x4,scale:s}; p_out=s*(R*p_in+t).')
    parser.add_argument('--image-map',help='TUM timestamp-string to real image filename JSON map.')
    parser.add_argument('--intrinsics',help='Explicit intrinsics JSON for Nerfstudio output.')
    parser.add_argument('--output',help='New JSON file; existing files are not overwritten.')
    args=parser.parse_args(argv)
    try:
        if args.input_format=='colmap-images' and (args.input_direction!='w2c' or args.input_axes!='opencv'):
            raise ValueError('COLMAP image poses are w2c/opencv; conflicting declarations are rejected.')
        frames,metadata=read_poses(args.input,args.input_format,load(args.image_map) if args.image_map else None)
        world,scale=similarity(load(args.world_similarity) if args.world_similarity else
                               load(args.dataparser_transform) if args.dataparser_transform else None,
                               bool(args.dataparser_transform))
        frames=convert(frames,args.input_direction,args.input_axes,args.output_direction,args.output_axes,world,scale)
        emit(output_document(frames,args.output_direction,args.output_axes,args.format,metadata,
                             load(args.intrinsics) if args.intrinsics else None),args.output)
        return 0
    except (OSError,ValueError,TypeError,KeyError,AttributeError) as exc:
        print(f'Input/report error: {exc}',file=sys.stderr)
        return 2
