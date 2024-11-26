import cv2

def resizeWithAspectRatio(image, width=None, height=None, inter=cv2.INTER_AREA):
    """
    # fun from stack overflow to resize imgs to for example the user screen, minor edits
    credits:
    https://stackoverflow.com/questions/35180764/opencv-python-image-too-big-to-display
    """
    dim = None
    (h, w) = image.shape[:2]

    if width is None and height is None:
        return image
    if width is None:
        r = height / float(h)
        dim = (int(w * r), height)
    else:
        r = width / float(w)
        dim = (width, int(h * r))

    return cv2.resize(image, dim, interpolation=inter)

def dummy():
    return 0

def printNBL(*args, **kwargs):
    """Prints Non Breaking Line, better name pending"""
    kwargs.setdefault('end', '')
    kwargs.setdefault('flush', True)
    return print(*args,**kwargs)