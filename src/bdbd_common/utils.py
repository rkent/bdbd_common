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
        count = 0
        for var in v:
            if n_per_line is not None:
                if count % n_per_line == 0:
                    if count == 0:
                        s += '\n['
                    else:
                        s += '\n '
            else:
                if count == 0:
                    s += '['
            s += '{}'.format(fstr(var, fmat, n_per_line))
            count += 1
        s += ']'
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
