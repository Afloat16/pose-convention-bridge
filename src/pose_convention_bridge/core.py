"""Explicit camera basis changes and world similarities; no convention guessing."""
from decimal import Decimal, InvalidOperation
import math
from pathlib import Path
from .common import integer, load, number, text


def identity(): return [[float(i==j) for j in range(4)] for i in range(4)]


def multiply(a,b): return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def rigid(matrix):
    if not isinstance(matrix,list) or len(matrix) not in {3,4} or any(not isinstance(row,list) or len(row)!=4 for row in matrix):
        raise ValueError('Pose matrix must be 3x4 or 4x4.')
    M=[[number(value,'matrix value') for value in row] for row in matrix]
    if len(M)==3: M.append([0.,0.,0.,1.])
    if any(abs(M[3][j]-float(j==3))>1e-8 for j in range(4)):
        raise ValueError('Homogeneous bottom row must be [0,0,0,1].')
    R=[row[:3] for row in M[:3]]
    if any(abs(sum(R[i][k]*R[j][k] for k in range(3))-float(i==j))>1e-6 for i in range(3) for j in range(3)):
        raise ValueError('Rotation is not orthonormal; scale/shear is not a rigid pose.')
    determinant=(R[0][0]*(R[1][1]*R[2][2]-R[1][2]*R[2][1])-
                 R[0][1]*(R[1][0]*R[2][2]-R[1][2]*R[2][0])+
                 R[0][2]*(R[1][0]*R[2][1]-R[1][1]*R[2][0]))
    if abs(determinant-1)>1e-6: raise ValueError('Rotation must be right-handed (determinant +1).')
    return M


def inverse(M):
    M=rigid(M); result=identity()
    for i in range(3):
        for j in range(3): result[i][j]=M[j][i]
        result[i][3]=-sum(M[j][i]*M[j][3] for j in range(3))
    return result


def quaternion(values,order):
    if len(values)!=4 or order not in {'wxyz','xyzw'}: raise ValueError('Quaternion needs an explicit wxyz or xyzw order.')
    q=[number(float(value) if isinstance(value,str) else value,'quaternion') for value in values]
    if order=='xyzw': q=[q[3],*q[:3]]
    norm=math.sqrt(sum(value*value for value in q))
    if abs(norm-1)>1e-3: raise ValueError('Quaternion must be unit length; normalization is not used to hide invalid input.')
    w,x,y,z=[value/norm for value in q]
    M=identity()
    M[:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),0.],
           [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),0.],
           [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y),0.]]
    return rigid(M)


def read_text(path):
    with Path(path).open('rb') as stream: raw=stream.read(64*1024*1024+1)
    if len(raw)>64*1024*1024: raise ValueError('Input exceeds 64 MiB.')
    return raw.decode('utf-8-sig')


def read_poses(path,input_format='json',image_map=None):
    frames=[]; metadata={}
    if input_format=='json':
        data=load(path)
        if not isinstance(data,dict) or not isinstance(data.get('frames'),list):
            raise ValueError('JSON must contain a frames list.')
        metadata={key:value for key,value in data.items() if key!='frames'}
        for index,frame in enumerate(data['frames']):
            if not isinstance(frame,dict): raise ValueError('Frame must be an object.')
            source=frame.get('transform_matrix',frame.get('matrix'))
            row=dict(frame,id=str(frame.get('id',frame.get('file_path',index))),matrix=rigid(source))
            row.pop('transform_matrix',None); frames.append(row)
    elif input_format=='tum':
        for line in read_text(path).splitlines():
            if not line.strip() or line.lstrip().startswith('#'): continue
            fields=line.split()
            if len(fields)!=8: raise ValueError('TUM row needs timestamp tx ty tz qx qy qz qw.')
            try:
                stamp=Decimal(fields[0])
                if not stamp.is_finite(): raise ValueError('TUM timestamp must be finite.')
            except InvalidOperation as exc: raise ValueError('Invalid TUM timestamp.') from exc
            M=quaternion(fields[4:8],'xyzw')
            for i,value in enumerate(fields[1:4]): M[i][3]=number(float(value),'translation')
            row={'id':fields[0],'timestamp':fields[0],'matrix':M}
            if image_map is not None:
                if fields[0] not in image_map: raise ValueError('Image map lacks a TUM timestamp; filenames are never invented.')
                row['file_path']=text(image_map[fields[0]],'image filename')
            frames.append(row)
    elif input_format=='colmap-images':
        rows=read_text(path).splitlines(); cursor=0
        while cursor<len(rows):
            line=rows[cursor]; cursor+=1
            if not line.strip() or line.lstrip().startswith('#'): continue
            fields=line.split(maxsplit=9)
            if len(fields)!=10: raise ValueError('Invalid COLMAP image pose line.')
            M=quaternion(fields[1:5],'wxyz')
            for i,value in enumerate(fields[5:8]): M[i][3]=number(float(value),'translation')
            while cursor<len(rows) and rows[cursor].lstrip().startswith('#'): cursor+=1
            if cursor>=len(rows): raise ValueError('COLMAP image is missing its observation line.')
            observations=rows[cursor].split(); cursor+=1
            if len(observations)%3: raise ValueError('Invalid COLMAP observation triples.')
            frames.append({'id':fields[0],'file_path':text(fields[9],'image filename'),'matrix':M})
    else: raise ValueError('Unsupported input format.')
    if not frames: raise ValueError('No camera poses found.')
    if len(frames)>100_000: raise ValueError('Frame limit of 100,000 exceeded.')
    ids=[frame['id'] for frame in frames]
    if len(set(ids))!=len(ids): raise ValueError('Duplicate pose identifiers.')
    return frames,metadata


def similarity(value=None,dataparser=False):
    if value is None: return identity(),1.0
    if not isinstance(value,dict): raise ValueError('World transform must be an object.')
    M=rigid(value.get('transform') if dataparser else value.get('matrix'))
    scale=number(value.get('scale',1.0),'world scale')
    if scale<=0: raise ValueError('World scale must be positive.')
    if dataparser:
        # Nerfstudio applies scale after its rigid transform, including translation.
        for i in range(3): M[i][3]*=scale
    return M,scale


def convert(frames,input_direction,input_axes,output_direction='c2w',output_axes='opengl',world=None,scale=1.0):
    if input_direction not in {'c2w','w2c'} or output_direction not in {'c2w','w2c'}:
        raise ValueError('Direction must be explicitly c2w or w2c.')
    if input_axes not in {'opencv','opengl'} or output_axes not in {'opencv','opengl'}:
        raise ValueError('Camera axes must be explicitly opencv or opengl.')
    world=rigid(world if world is not None else identity()); scale=number(scale,'scale')
    if scale<=0: raise ValueError('Scale must be positive.')
    basis=identity()
    if input_axes!=output_axes: basis[1][1]=basis[2][2]=-1.
    result=[]
    for frame in frames:
        M=rigid(frame['matrix'])
        if input_direction=='w2c': M=inverse(M)
        M=multiply(M,basis)
        mapped=multiply(world,M)
        for i in range(3):
            mapped[i][3]=scale*sum(world[i][j]*M[j][3] for j in range(3))+world[i][3]
        if output_direction=='w2c': mapped=inverse(mapped)
        result.append(dict(frame,matrix=rigid(mapped)))
    return result


INTRINSICS={'fl_x','fl_y','cx','cy','w','h','k1','k2','k3','k4','p1','p2','camera_model','is_fisheye'}


def output_document(frames,direction,axes,output_format='generic',metadata=None,intrinsics=None):
    if output_format=='generic':
        return {'schema_version':1,'direction':direction,'camera_axes':axes,'frames':frames,
                'world_note':'No automatic up-axis orientation, centering, calibration, or scene-point transformation.'}
    if output_format!='nerfstudio' or direction!='c2w' or axes!='opengl':
        raise ValueError('Nerfstudio output requires c2w and opengl.')
    base={key:value for key,value in (metadata or {}).items() if key in INTRINSICS}
    if intrinsics:
        if not isinstance(intrinsics,dict): raise ValueError('Intrinsics must be an object.')
        base.update({key:value for key,value in intrinsics.items() if key in INTRINSICS})
    output=[]
    for frame in frames:
        row={key:value for key,value in frame.items() if key in INTRINSICS}
        values=dict(base,**row)
        for key in ('fl_x','fl_y','cx','cy','w','h'):
            if key not in values: raise ValueError(f'Nerfstudio output requires explicit {key} intrinsics.')
            if key in {'w','h'}: integer(values[key],key,1)
            else:
                number(values[key],key)
                if key in {'fl_x','fl_y'} and values[key]<=0: raise ValueError('Focal lengths must be positive.')
        row.update(file_path=text(frame.get('file_path'),'frame.file_path'),transform_matrix=frame['matrix'])
        output.append(row)
    return dict(base,frames=output)
