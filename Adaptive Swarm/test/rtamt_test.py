import sys
import rtamt

def monitor():
    # a1 = [(0, 3), (3, 2)]
    # b1 = [(0, 2), (2, 5), (4, 1), (7, -7)]

    # a1 = [(0, 0), (1, 1.4), (5, 7)]
    # b1 = [(0, 0), (1, 1), (8, 9)]

    # a2 = [(5, 6), (6, -2), (8, 7), (11, -1)]
    # b2 = [(10, 4)]

    # a3 = [(13, -6), (15, 0)]
    # b3 = [(15, 0)]

    a1 = [(0, 0)]
    b1 = [(0, 0)]

    a2 = [(1, 1)]
    b2 = [(1, 0.5)]

    # # stl
    spec = rtamt.STLDenseTimeSpecification()
    spec.name = 'STL dense-time specification'
    spec.declare_var('a', 'float')
    spec.declare_var('b', 'float')
    spec.spec = '(a>=b)'
    try:
        spec.parse()
    except rtamt.RTAMTException as err:
        print('RTAMT Exception: {}'.format(err))
        sys.exit()

    rob = spec.update(['a', a1], ['b', b1])
    print('rob: ' + str(rob))

    rob = spec.update(['a', a2], ['b', b2])
    print('rob: ' + str(rob))

    # rob = spec.update(['a', a2])
    # print('rob: ' + str(rob))

    # rob = spec.update(['a', a3])
    # print('rob: ' + str(rob))

if __name__ == '__main__':
    monitor()