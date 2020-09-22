def fstr(v, fmat='6.3f'):
    # generate an output string with floats formatted using fmat
    s = ''
    if isinstance(v, float):
        s += ("{:" + fmat + "} ").format(v)
    elif isinstance(v, dict):
        s += '{ '
        for key, var in v.items():
            s += '{}: {}'.format(key, fstr(var, fmat))
        s += '} '
    elif isinstance(v, list):
        s += '['
        for var in v:
            s += '{}'.format(fstr(var, fmat))
        s += '] '
    elif isinstance(v, tuple):
        s += '('
        for var in v:
            s += '{}'.format(fstr(var, fmat))
        s += ') '
    elif isinstance(v, int):
        s += '{} '.format(v)
    else:
        try:
            f = float(v)
            s += ("{:" + fmat + "} ").format(f)
        except:
            s += '{} '.format(v)
    return s

