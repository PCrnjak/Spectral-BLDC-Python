import struct


int_to_3_bytes = struct.Struct('>I').pack # BIG endian order

# Fuses 3 bytes to 1 signed int
def fuse_3_bytes(var_in):
    value = struct.unpack(">I", bytearray(var_in))[0] # converts bytes array to int

    # convert to negative number if it is negative
    if value >= 1<<23:
        value -= 1<<24

    return value

# Fuses 2 bytes to 1 signed int
def fuse_2_bytes(var_in):
    value = struct.unpack(">I", bytearray(var_in))[0] # converts bytes array to int

    # convert to negative number if it is negative
    if value >= 1<<15:
        value -= 1<<16

    return value

# Fuses 4 bytes to 1 signed int
def fuse_4_bytes(bytes_array):
    # Unpack 4 bytes into an unsigned integer
    value = struct.unpack(">I", bytearray(bytes_array))[0]
 # Convert to a negative number if it is negative
    if value & 0x80000000:
        value -= 1 << 32

    return value

# Fuses 4 bytes to 1 32bit float value
def fuse_float_bytes(bytes_array):
    # Unpack 4 bytes into a float using IEEE 754 single-precision format (32 bits)
    float_value = struct.unpack('>f', bytearray(bytes_array))[0]
    return float_value


# Split data to 3 bytes 
def split_2_3_bytes(var_in):
    y = int_to_3_bytes(var_in & 0xFFFFFF) # converts my int value to bytes array
    return y[1:4]

# Splits data to 2 bytes
def split_2_2_bytes(var_in):
    return struct.pack(">H", var_in & 0xFFFF)

# Splits int data to 4 bytes
def split_2_4_bytes(var_in):
    return struct.pack(">I", var_in & 0xFFFFFFFF)

# Splits float data to 4 bytes
def float_to_bytes(float_value):
    # Pack the float value into 4 bytes using IEEE 754 single-precision format (32 bits)
    return struct.pack('>f', float_value)

def extract_from_can_id(can_id):
    # Extracting ID2 (first 4 MSB)
    id2 = (can_id >> 7) & 0xF

    # Extracting CAN Command (next 6 bits)
    can_command = (can_id >> 1) & 0x3F

    # Extracting Error Bit (last bit)
    error_bit = can_id & 0x1
    
    return id2, can_command, error_bit


def combine_2_can_id(id2, can_command, error_bit):
    # Combine components into an 11-bit CAN ID
    can_id = 0

    # Add ID2 (first 4 MSB)
    can_id |= (id2 & 0xF) << 7

    # Add CAN Command (next 6 bits)
    can_id |= (can_command & 0x3F) << 1

    # Add Error Bit (last bit)
    can_id |= (error_bit & 0x1)

    return can_id

# Fuse bitfield list to byte
def fuse_bitfield_2_bytearray(var_in):
    number = 0
    for b in var_in:
        number = (2 * number) + b
    return bytes([number])

# Splits byte to bitfield list
def split_2_bitfield(var_in):
    #return [var_in >> i & 1 for i in range(7,-1,-1)] 
    return [(var_in >> i) & 1 for i in range(7, -1, -1)]

# Combine 2 bits into int
def combine_bits(bit1, bit2):
    # Ensure that bit1 and bit2 are either 0 or 1
    bit1 = 1 if bit1 else 0
    bit2 = 1 if bit2 else 0
    
    # Combine the bits using bitwise OR
    result = (bit1 << 1) | bit2
    return result

if __name__ == "__main__":
    # Packing data to send on can bus
    pos_ = -150
    speed_ = -187
    torque_ = -3047
    a = split_2_3_bytes(pos_)
    b = split_2_3_bytes(speed_)
    c = split_2_2_bytes(torque_)
    print(a)
    print(b)
    print(c)

    # Unpacking data we receive from can bus
    message = bytearray(b'\xff\xffj\xff\xffE\x0b\xe7')
    print(message)
    position = fuse_3_bytes(b'\x00' + message[0:3])
    speed = fuse_3_bytes( b'\x00' + message[3:6])
    torque = fuse_2_bytes(b'\x00'+ b'\x00' + message[6:8])
    print(position)
    print(speed)
    print(torque)



    