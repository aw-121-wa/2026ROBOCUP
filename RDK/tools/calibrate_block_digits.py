"""Import full ball-camera snapshots; never move the robot or claim live accuracy."""
import argparse
from pathlib import Path
import sys
import cv2
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from rdk_vision.block_digits import BlockDetector, EMPTY


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side',choices=('red','blue'),required=True)
    parser.add_argument('--row',type=int,choices=(1,2,3),required=True)
    parser.add_argument('--input',type=Path,help='Full 640x480 snapshot from ball camera at the matching action pose')
    parser.add_argument('--label',choices=('empty','1','2','3'))
    parser.add_argument('--roi',type=int,nargs=4,metavar=('X','Y','W','H'))
    parser.add_argument('--glyph',type=int,nargs=4,help='Digit-only rectangle in full-image coordinates; required for digit samples')
    parser.add_argument('--enable',action='store_true',help='Validate all four labeled snapshots, then enable this row only')
    parser.add_argument('--config',type=Path,default=ROOT/'rdk_vision'/'block_digits.yaml')
    args=parser.parse_args(argv)
    root=args.config.parent
    data=yaml.safe_load(args.config.read_text(encoding='utf-8'))
    settings=data[args.side][args.row]
    if args.enable and (args.input or args.roi):parser.error('--enable must be run separately after sample import')
    if args.roi:
        if settings.get('calibrated'):parser.error('disable this row before changing its ROI')
        settings['roi']=args.roi
    BlockDetector(root,dict(settings,calibrated=False))  # Validate ROI before writing.
    samples=root/settings['templates'];samples.mkdir(parents=True,exist_ok=True)
    if args.input:
        if args.label is None:parser.error('--input requires --label')
        frame=cv2.imread(str(args.input))
        if frame is None or frame.shape[:2]!=(480,640):parser.error('input must be a valid 640x480 ball-camera snapshot')
        x,y,w,h=settings['roi'];crop=frame[y:y+h,x:x+w]
        if args.label=='empty':
            output=root/settings['empty_reference']
            sample=cv2.cvtColor(crop,cv2.COLOR_BGR2GRAY)
        else:
            if not args.glyph:parser.error('digit import requires a tight --glyph rectangle containing only the printed digit')
            a,b,c,d=args.glyph
            if a<x or b<y or c<=0 or d<=0 or a+c>x+w or b+d>y+h:parser.error('glyph must be fully inside the row ROI')
            sample=cv2.cvtColor(frame[b:b+d,a:a+c],cv2.COLOR_BGR2GRAY)
            _,sample=cv2.threshold(sample,0,255,cv2.THRESH_BINARY|cv2.THRESH_OTSU)
            output=samples/(args.label+'.png')
        if not cv2.imwrite(str(output),sample):raise OSError(f'cannot save {output}')
        if not cv2.imwrite(str(samples/(args.label+'-scene.png')),frame):raise OSError('cannot save validation scene')
        settings['calibrated']=False  # Any sample change requires validation again.
    elif not args.enable:parser.error('provide --input or --enable')
    if args.enable:
        detector=BlockDetector(root,dict(settings,calibrated=True))
        for label,expected in (('empty',EMPTY),('1',1),('2',2),('3',3)):
            image=cv2.imread(str(samples/(label+'-scene.png')))
            if image is None or detector.detect(image)!=expected:
                raise ValueError(f'validation failed for {label}; keep calibrated=false and adjust samples/ROI')
        settings['calibrated']=True
        print('Labeled-snapshot validation passed. This does not replace live validation in all three columns.')
    args.config.write_text(yaml.safe_dump(data,sort_keys=False,allow_unicode=True),encoding='utf-8')
    print(f'{args.side} row {args.row}: calibrated={settings["calibrated"]}')

if __name__=='__main__':main()
