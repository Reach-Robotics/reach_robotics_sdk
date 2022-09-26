from bplprotocol import BPLProtocol, PacketID



if __name__ == '__main__':
    print(list(BPLProtocol.encode_packet(0x01, 0x02, BPLProtocol.encode_floats([3.123]))))
    print(list( BPLProtocol.encode_floats([3.123])))

    print(list(BPLProtocol.encode_floats([1.1, 2.2, 3.0])))