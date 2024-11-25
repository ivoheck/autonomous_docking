
def get_wheel_speed(v_x,v_y,v_z):
    #a = 10.35 #20.7 /2
    #b = 18.75
    c = 29.1

    # w in rad/s
    #w_1 = (1/radius) * (v_x - v_y -(a+b)*v_z)
    #w_2 = (1/radius) * (v_x + v_y +(a+b)*v_z)
    #w_3 = (1/radius) * (v_x + v_y -(a+b)*v_z)
    #w_4 = (1/radius) * (v_x - v_y +(a+b)*v_z)

    # w in m/s
    w_1 =  (v_x - v_y -(c)*v_z)
    w_2 =  (v_x + v_y +(c)*v_z)
    w_3 =  (v_x + v_y -(c)*v_z)
    w_4 =  (v_x - v_y +(c)*v_z)
    
    return w_1,w_2,w_3,w_4

def get_platform_speed(w_1,w_2,w_3,w_4):

    #a = 10.35 #20.7 /2
    #b = 18.75
    c = 29.1

    # mit w als rad/s
    #v_x = (radius/4) * (w_1 + w_2 + w_3 + w_4)
    #v_y = (radius/4) * (-w_1 + w_2 + w_3 - w_4)
    #v_z = (radius/4) * (-(w_1/(a+b)) + (w_2/(a+b)) - (w_3/(a+b)) + (w_4/(a+b)))

    # mit w als m/s
    v_x = (1/4) * (w_1 + w_2 + w_3 + w_4)
    v_y = (1/4) * (-w_1 + w_2 + w_3 - w_4)
    v_z = (1/4) * (-(w_1/(c)) + (w_2/(c)) - (w_3/(c)) + (w_4/(c)))

    return v_x,v_y,v_z