def a(name, data_class, *args, **kwargs):
    timeout = 45.0 if 'timeout' not in kwargs else kwargs['timeout']
    print(name, data_class, timeout, args, kwargs)
    return aa(name, data_class, *args, **kwargs)

def aa(b1, b2, *args, **kwargs):
    print(b1, b2, args, kwargs)
    return b1

print(a('1', '2', 'my_cb', timeout=10.0, stuff='me'))

