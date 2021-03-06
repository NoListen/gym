def q_inv(a):
    return [a[0], -a[1], -a[2], -a[3]]

def q_mult(a, b): # multiply two quaternion
    w = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3]
    i = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2]
    j = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1]
    k = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
    return [w, i, j, k]

def arg(name, type=None, help=None, nargs=None, mapper=None, choices=None,
        prefix=True):
    def wrap(fn):
        assert fn.__name__ == '__init__'
        if not hasattr(fn, '_autoargs_info'):
            fn._autoargs_info = dict()
        fn._autoargs_info[name] = dict(
            type=type,
            help=help,
            nargs=nargs,
            choices=choices,
            mapper=mapper,
        )
        return fn
    return wrap