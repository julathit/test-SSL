import numpy as np

# S is vector that tell distace to go and oretation to make <dx,dy,dom>
# V is vector speed each compo is <vx,vy,om> speed of robot
# <fox,fxy> is front vector <sox,soy> is side vector
# M is unit orthogonal matrix of [[fox,sox,0],
#                                 [foy,s0y,0], 
#                                 [0,  0  ,1]]
# Remid yoy if M is unit orthogonal MT = M^-1
#o is ang of robot


def SToV(S: np.array,o: np.array) -> np.array:
    fox,foy = np.cos(o),np.sin(o)
    sox, soy = (-foy), (fox)
    M = np.array([[fox,sox,0],
                 [foy,soy,0],
                 [0  ,0  ,1]])
    Mmal = lambda x, y: np.matmul(x, y)
    MT = np.transpose(M)
    V = Mmal(MT,S)
    return V