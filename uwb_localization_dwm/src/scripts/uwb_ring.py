#!/usr/bin/env python3

'''
A ring buffer use for storing a fix number of uwb distances.
'''

class ring_buffer:
    """ class that implements a not-yet-full buffer """
    def __init__(self,maxlen):
        self.maxlen = maxlen
        self.data = []

    class __Full:
        def append(self, x):
            self.data[self.cur] = x
            self.cur = (self.cur+1) % self.maxlen
        def get(self):
            return self.data[self.cur:]+self.data[:self.cur]

    def append(self,x):
        self.data.append(x)
        if len(self.data) == self.maxlen:
            self.cur = 0
            self.__class__ = self.__Full

    def get(self):
        """ Return a list of elements from the oldest to the newest. """
        return self.data

def main():
    x=ring_buffer(5)
    x.append(1); x.append(2); x.append(3); x.append(4)
    print(x.__class__, x.get())
    x.append(5)
    print(x.__class__, x.get())
    x.append(6)
    print(x.data, x.get())
    x.append(7); x.append(8); x.append(9); x.append(10)
    print(x.data, x.get())

# example usage
if __name__=='__main__':
    main()
