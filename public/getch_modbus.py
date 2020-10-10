import orienbus
import getch
import time

port = '/dev/ttyUSB0' # modbus port 

# Create orienbus object with port name
orienbus = orienbus.OrienBus(port)

# slave addresses
address_rot_lb = 13
address_rot_rb = 12
address_rot_lf = 11
address_rot_rf = 10

address_dr_lb = 4
address_dr_rb = 3
address_dr_lf = 2
address_dr_rf = 1

ls_address = [
    address_rot_lb,
    address_rot_rb,
    address_rot_lf,
    address_rot_rf,
    address_dr_lb,
    address_dr_rb,
    address_dr_lf,
    address_dr_rf,
]


# Initialze motors with slave addresses
m_rot_lb = orienbus.initialize(address_rot_lb)
m_rot_rb = orienbus.initialize(address_rot_rb)
m_rot_lf = orienbus.initialize(address_rot_lf)
m_rot_rf = orienbus.initialize(address_rot_rf)

m_dr_lb = orienbus.initialize(address_dr_lb)
m_dr_rb = orienbus.initialize(address_dr_rb)
m_dr_lf = orienbus.initialize(address_dr_lf)
m_dr_rf = orienbus.initialize(address_dr_rf)

def mode_rot_f(addresses, speed):
    for i in addresses:
        i.writeSpeed(speed)

def mode_turn_f(addresses, speed):
    addresses[-1].writeSpeed(speed/2)
    for i in addresses[: -1]:
        i.writeSpeed(speed)

ls_back_rot = [m_rot_lb,m_rot_rb,]
ls_front_rot = [m_rot_lf,m_rot_rf]

#ls_mot_trans = [m_dr_lb,m_dr_lf, m_dr_rb, m_dr_rf]
ls_mot_trans = [m_dr_lf, m_dr_rf, m_dr_rb, m_dr_lb]
#ls_mot_rot = [m_rot_lb,m_rot_lf, m_rot_rb, m_rot_rf]
ls_mot_rot = [m_rot_lf, m_rot_rf, m_rot_rb, m_rot_lb]

run = True

while run:

    char = getch.getch()

    if char == 'r':     # move backward
        mode_rot_f(ls_mot_trans,300)

    if char == 't':
        mode_rot_f(ls_mot_trans,-300) # move forward

    if char == 'y':
        mode_rot_f(ls_mot_trans,0)

    if char == 'q':
        mode_turn_f(ls_mot_rot,250) # clockwise

    if char == 'w': # be anti-clockwise
        mode_turn_f(ls_mot_rot,-250)
    
    if char == 'e':
        mode_turn_f(ls_mot_rot,0)

    if char == 's': # emergency stop / stop all
        mode_rot_f(ls_mot_trans,0)
        mode_turn_f(ls_mot_rot,0)

    if char ==  'p':
        run = False
    

    
