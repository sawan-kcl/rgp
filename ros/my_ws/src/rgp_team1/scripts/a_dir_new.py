class Dir:
    def __init__(self,obj_t,x,y,z,h,b,l,q1,q2,q3,q4,state=True):
        self.obj_t = obj_t
        self.x=x
        self.y=y
        self.z=z
        self.l=l
        self.b=b
        self.h=h
        self.q1=q1
        self.q2=q2
        self.q3=q3
        self.q4=q4
        self.state = state


table1=Dir('table',0.1400, 0.000000, 0.000000, 0, 0, 0, 0, 0, 0.7068, 0.7074)
table2=Dir('table',0.9400, 0.000000, 0.000000, 0, 0, 0, 0, 0, 0.7068,0.7074)
unit_box=Dir('unit_box',-0.1400, 0.000000, 0.565000, 0, 0, 0, 0, 0, 0, 1)
BlueBin=Dir('bin',-0.4652, -0.500000, 1.000000,0,0,0,0,0,-0.7068, 0.7074)
GreenBin=Dir('bin',-0.4654, 0.500000,  1.000000,0,0,0,0,0,-0.7068, 0.7074)
panda=Dir('panda',0, 0, 0,0,0,0,0, 0, 0, 0.9999)
wcase=Dir('case',0.4299, -0.6799, 0.5774,0,0,0,-0.42226, -0.5671, -0.4222, 0.5672)
box1=Dir('carton_box',-0.0922, 0.3710, 0.5689,0,0,0,0, 0, 0.0399, 0.9992)
box2=Dir('carton_box',0.2200, 0.3550, 0.5940,0,0,0,0.7015, -0.0881, -0.0881, 0.7016)
scale1=Dir('scale',0.5600, 0.3672, 0.5450,0,0,0,0,0,-0.3428,0.9393)
pouch1=Dir('pouch',0.4340, -0.0330, 0.5240, 0,0,0, 0, 0, 0, 0.9999)
pouch2=Dir('pouch',0.4330, 0.0410, 0.5240, 0,0,0, 0, 0, 0, 0.9999)
pouch3=Dir('pouch', 0.5210, -0.0310, 0.5240, 0,0,0, 0, 0, 0, 0.9999)
pouch4=Dir('pouch',0.5220, 0.0420, 0.5240, 0, 0, 0, 0, 0, 0, 0.9999)
pouch5=Dir('pouch',0.7860, 0.2040, 0.5240,0,0,0, 0, 0, 0.1246, 0.9922)
pouch6=Dir('pouch',0.7660, 0.2840, 0.5240,0,0,0, 0, 0, 0.1494, 0.9887)
pouch7=Dir('pouch', 0.5209, -0.1310, 0.524,0,0,0, -0.0017, -0.0039, 0.4794, 0.8775)
pouch8=Dir('pouch',0.4330, 0.1310, 0.5240,0,0,0, 0, 0, 0.3894, 0.9211)
gcan1=Dir('can',-0.1070, 0.3628, 0.6819, 0.1062, 0.06, 0.06,0.0045, -0.0052, 0, 0.9999, True)
gcan2=Dir('can',0.2216, -0.0219, 0.5478, 0.1062, 0.06, 0.06,0.5261, -0.4724, 0.4773, 0.5217, False)
gcan3=Dir('can',-0.0791, -0.6619, 0.5733, 0.1062, 0.06, 0.06,-0.0010, -0.0017, -0.7313, 0.6820, True)
gcan4=Dir('can', 0.3690, -0.5295, 0.5904, 0.1062, 0.06, 0.06,0, 0, -0.9971, 0.0757, True)
rcan1=Dir('can',0.2953, 0.5024, 0.5731, 0.1062, 0.06, 0.06,0.07073, 0.9975, 0.0045, 0.0028, True)
rcan2=Dir('can',-0.0798, -0.6603, 0.6890, 0.1062, 0.06, 0.06,0.9988, 0.0464, 0.0152, 0.0043, True)
rcan3=Dir('can',0.6993, 0.0323, 0.5731, 0.1062, 0.06, 0.06,0.9460, 0.3240, 0.0043, 0.0027, True)
ycan1=Dir('can',0.5993, 0.1718, 0.5730, 0.1062, 0.06, 0.06,0.7960, 0.6051, 0.0054, 0.0013, True)
ycan2=Dir('can',0.5498, -0.4820, 0.5648, 0.1062, 0.06, 0.06, -0.6322, 0.3165, -0.2531, 0.6603, False)
ycan3=Dir('can',0.3600, -0.4499, 0.5904, 0.1062, 0.06, 0.06, 0, 0, -0.9954, 0.0956, True)
ycan4=Dir('can',0.2599, -0.4197, 0.5648, 0.1062, 0.06, 0.06, 0.04201, -0.7058, 0.3697, 0.6027, False)
rBottle1=Dir('bottle',0.2200, 0.6191, 0.5487, 0.174, 0.06, 0.06, 0.6037, 0.3680, 0.5881, 0.3925, False)
rBottle2=Dir('bottle', 0.8196, 0.0000, 0.5485, 0.174, 0.06, 0.06, 0.7056, -0.0374, 0.0374, 0.7065, False)
bBottle1=Dir('bottle',-0.1703, -0.4603, 0.6135, 0.174, 0.06, 0.06,0.001, -0.0007, -0.0009, 0.9999, True)
bBottle2=Dir('bottle',0.0732, 0.2299, 0.6135, 0.174, 0.06, 0.06, 0, 0, -0.0004, 0.9999, True)
bBottle3=Dir('bottle',0.2630, -0.2278, 0.5488, 0.174, 0.06, 0.06,0.0280, 0.7068, 0.0280, 0.7062, False)
yBottle1=Dir('bottle',-0.1430, 0.5260, 0.5485, 0.174, 0.06, 0.06, 0.0338, 0.7061, 0.0409, 0.7061, False)
yBottle2=Dir('bottle',0.6797, -0.1602, 0.6135, 0.174, 0.06, 0.06, 0.0002, -0.0005, -0.0092,0.9999, True)
yBottle3=Dir('bottle',0.30636, 0.1509, 0.6135, 0.174, 0.06, 0.06, 0, 0, -0.0043,0.9999, True)
yBottle4=Dir('bottle',0.2427,-0.6250, 0.5658, 0.174, 0.06, 0.06, 0.0313, 0.7064, 0.0382, 0.7060, False)
