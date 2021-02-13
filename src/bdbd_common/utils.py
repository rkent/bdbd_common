import inspect

def reach(name) :
    # https://stackoverflow.com/questions/15608987/access-variables-of-caller-function-in-python
    for f in inspect.stack() :
        if name in f[0].f_locals : return f[0].f_locals[name]
    return None 

def sstr(vs, **kwargs):
    dd = dict()
    for v in vs.split(' '):
        dd[v] = reach(v)
    return fstr(dd, **kwargs)

def fstr(v, fmat='6.3f', n_per_line=None):
    # generate an output string with floats formatted using fmat
    s = ''
    if isinstance(v, float):
        s += ("{:" + fmat + "} ").format(v)
    elif isinstance(v, dict):
        s += '{ '
        for key, var in v.items():
            s += '{}: {}'.format(key, fstr(var, fmat, n_per_line))
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
