#!/usr/bin/env python3
import sys
import numpy as np


def write_pfm16(file, image, scale=1):
    """ Write a pfm file """
    file = open(file, 'wb')

    image = image.astype(np.float16)

    color = None

    if image.dtype.name != 'float16':
        raise Exception('Image dtype must be float16.')

    if len(image.shape) == 3 and image.shape[2] == 3: # color image
        color = True
    elif len(image.shape) == 2 or len(image.shape) == 3 and image.shape[2] == 1: # grayscale
        color = False
    else:
        raise Exception('Image must have H x W x 3, H x W x 1 or H x W dimensions.')

    file.write('PF\n'.encode('utf-8')  if color else 'Pf\n'.encode('utf-8'))
    temp_str = '%d %d\n' % (image.shape[1], image.shape[0])
    file.write(temp_str.encode('utf-8'))

    endian = image.dtype.byteorder

    if endian == '<' or endian == '=' and sys.byteorder == 'little':
        scale = -scale

    temp_str = '%f\n' % scale
    file.write(temp_str.encode('utf-8'))

    image.tofile(file)

def write_exr(file, image, scale=1, floattype=np.float32):
    """ Write a pfm file """
    file = open(file, 'wb')

    image = image.astype(floattype)
    imageio.imwrite('float_img.exr', image)


def readPF(filename):
    """Read named PF file into Numpy array"""
    import re
    import cv2
    import numpy as np
    from PIL import Image

    # Slurp entire file into memory as binary 'bytes'
    with open(filename, 'rb') as f:
        data = f.read()

    # Check correct header, return None if incorrect
    if not re.match(b'Typ=Pic98::TPlane<float>', data):
        return None
   
    # Get Lines and Columns, both must be present, else return None
    L = re.search(b'Lines=(\d+)',   data)
    C = re.search(b'Columns=(\d+)', data)
    if not (L and C):
        return None
    height = int(L.groups()[0])
    width  = int(C.groups()[0])
    #print(f"DEBUG: Height={:height}, width={:width} ".format())

    # Take the data from the END of the file in case other header lines added at start
    na = np.frombuffer(data[-4*height*width:], dtype=np.dtype('<f4')).reshape((height,width))

    # Some debug stuff
    min, max, mean = na.min(), na.max(), na.mean()
    #print(f"DEBUG: min={min}, max={max}, mean={mean}")

    return na

from _ctypes import PyObj_FromPtr  # see https://stackoverflow.com/a/15012814/355230
import json
import re

# from https://stackoverflow.com/questions/42710879/write-two-dimensional-list-to-json-file
class NoIndent(object):
    """ Value wrapper. """
    def __init__(self, value):
        if not isinstance(value, (list, tuple)):
            raise TypeError('Only lists and tuples can be wrapped')
        self.value = value

# from https://stackoverflow.com/questions/42710879/write-two-dimensional-list-to-json-file
class MyEncoder(json.JSONEncoder):
    FORMAT_SPEC = '@@{}@@'  # Unique string pattern of NoIndent object ids.
    regex = re.compile(FORMAT_SPEC.format(r'(\d+)'))  # compile(r'@@(\d+)@@')

    def __init__(self, **kwargs):
        # Keyword arguments to ignore when encoding NoIndent wrapped values.
        ignore = {'cls', 'indent'}

        # Save copy of any keyword argument values needed for use here.
        self._kwargs = {k: v for k, v in kwargs.items() if k not in ignore}
        super(MyEncoder, self).__init__(**kwargs)

    def default(self, obj):
        return (self.FORMAT_SPEC.format(id(obj)) if isinstance(obj, NoIndent)
                    else super(MyEncoder, self).default(obj))

    def iterencode(self, obj, **kwargs):
        format_spec = self.FORMAT_SPEC  # Local var to expedite access.

        # Replace any marked-up NoIndent wrapped values in the JSON repr
        # with the json.dumps() of the corresponding wrapped Python object.
        for encoded in super(MyEncoder, self).iterencode(obj, **kwargs):
            match = self.regex.search(encoded)
            if match:
                id = int(match.group(1))
                no_indent = PyObj_FromPtr(id)
                json_repr = json.dumps(no_indent.value, **self._kwargs)
                # Replace the matched id string with json formatted representation
                # of the corresponding Python object.
                encoded = encoded.replace(
                            '"{}"'.format(format_spec.format(id)), json_repr)

            yield encoded


def exist_and_not_empty(filepath):
    try:
        import pathlib as p
        path = p.Path(filepath)
        if '~' in filepath:
            path = path.expanduser()
        if not path.exists() and path.stat().st_size > 0:
            return False
        return True
    except FileNotFoundError:
        return False

if __name__ == '__main__':
    import numpy as np

    file = '/home/olaya/Documents/AirSim/dataset/henlo0/image_depth/05172022-14:29:11.pfm'
    na = readPF(file)

    ################################################################################
    # Use either of the following to save the image:
    ################################################################################
    # Save with OpenCV as scaled PNG
    u16 = (65535*(na - np.min(na))/np.ptp(na)).astype(np.uint16)  
    cv2.imwrite('OpenCV.png', u16)

    # Convert to PIL Image and save as TIFF
    pi = Image.fromarray(na, mode='F')
    pi.save('PIL.tif')

