from colav_interfaces.msg import UnsafeSet

def unsafe_set_example() -> UnsafeSet:
    """unsafe set example"""
    unsafe_set = UnsafeSet()
    
    unsafe_set.mission_tag = "mission_0001"
    
    unsafe_set.header.stamp.sec = 1000
    unsafe_set.header.stamp.nanosec = 20030
    unsafe_set.header.frame_id = "map"
    
    unsafe_set.vertices.data = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    
    return unsafe_set

def main():
    """main function"""
    unsafe_set = unsafe_set_example()
    print (unsafe_set)
    
if __name__ == "__main__":
    """main function"""
    main()