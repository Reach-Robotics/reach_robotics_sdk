
from rs_protocol import PacketID

class ExtendPacketID():
    ''' 
    Extends the PacketID class by calling PacketID.add() with new packet IDs and data types.


    In some cases, Reach Robotics may add new packet IDs to the protocol, or provide additional 
    PacketID's for advanced users. In these cases, the new packet IDs and their corresponding 
    data types can be added to this class without modifiying the rs_protocol library. 

    To use this class, simply import it in the main script to merge the extended packets. 
    '''
    PacketType = dict()

    # Add extended packet IDs here using the format: 
    #     <PACKET_NAME> = <PACKET_ID>
    #     PacketType[<PACKET_NAME>] = <DATA_TYPE>

PacketID.add(ExtendPacketID)
