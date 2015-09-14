from sslib import *
if __name__=='__main__':
    a=[0.000001*2**i for i in xrange(0,25)]
    t=[i for i in xrange(0,25)]
    plt.plot(a,'o')
    #plt.show()
    A = array([[0,0,1,0],
               [0,0,0,1],
               [0,0,0,0],
               [0,0,0,0]])
    print scipy.linalg.expm(A*0.3)
    
    #print time.time()-start
    #print a