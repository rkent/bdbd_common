import inspect

def reach(_name):
    # https://stackoverflow.com/questions/15608987/access-variables-of-caller-function-in-python
    for f in inspect.stack() :
        if _name in f[0].f_locals : return f[0].f_locals[_name]
    return None 

def dotreach(_name):
    vnames = _name.split('.')
    value = None
    for vname in vnames:
        if value is not None:
            value = getattr(value, vname)
        else:
            value = reach(vname)
        if inspect.isroutine(value):
            value = value()
    return value

def sstr(vs, **kwargs):
    dd = dict()
    for v in vs.split(' '):
        dd[v] = dotreach(v)
    return fstr(dd, **kwargs)

def fstr(v, fmat='6.3f', n_per_line=None):
    # generate an output string with floats formatted using fmat
    s = ''
    if isinstance(v, float):
        s += ("{:" + fmat + "} ").format(v)
    elif isinstance(v, dict):
        s += '{ '
        firstItem = True
        for key, var in v.items():
            if not firstItem:
                s += ','
            else:
                firstItem = False
            s += ' {}: {}'.format(key, fstr(var, fmat, n_per_line))
        s += '} '
    elif isinstance(v, list):
        s += '[ '
        count = 0
        for var in v:
            if n_per_line and count % n_per_line == 0:
                s += '\n  '
            s += '{}'.format(fstr(var, fmat, n_per_line))
            count += 1
        s += '] '
    elif isinstance(v, tuple):
        s += '('
        for var in v:
            s += '{}'.format(fstr(var, fmat, n_per_line))
        s += ') '
    elif isinstance(v, int):
        s += '{} '.format(v)
    elif hasattr(v, 'tolist'):
        # numpy
        s += '(np)' + fstr(v.tolist(), fmat, n_per_line)
    else:
        try:
            f = float(v)
            s += ("{:" + fmat + "} ").format(f)
        except:
            s += '{} '.format(v)
    return s

def gstr(v, fmat='9.3g', n_per_line=10):
    # a variation of fstr that works better with small numbers
    return fstr(v, fmat, n_per_line)

def getShape(o):
    s = ''
    if o is None:
        s += ' None '
    elif isinstance(o, str):
        s += ' "{}" '.format(o)
    elif isinstance(o, list) or isinstance(o, tuple):
        for i in range(len(o)):
            s += ' [{}] {} '.format(i, getShape(o[i]))
    elif hasattr(o, 'shape'):
        s += ' {} '.format(o.shape)
    elif hasattr(o, 'keys'):
        for key in o.keys():
            s += ' ["{}"] '.format(key) + getShape(o[key])
    elif hasattr(o, '__iter__'):
        for item in o:
            s += getShape(item)
    else:
        s += ' {} '.format(type(o))
    return s
