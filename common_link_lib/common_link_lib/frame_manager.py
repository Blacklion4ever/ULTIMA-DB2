class FrameManager:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: bytes):
        """Ajoute des données reçues et découpe les trames complètes."""
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 4:  # Minimum : header, subheader, taille, CRC
            length = self.buffer[2]
            if len(self.buffer) < length + 4:
                break
            frame = self.buffer[:length + 4]
            frames.append(frame)
            del self.buffer[:length + 4]
        return frames

    def parse_frame(self, frame: bytes):
        """Retourne un dict avec les infos décodées."""
        return {
            'header': frame[0],
            'subheader': frame[1],
            'length': frame[2],
            'payload': frame[3:-1],  # Exclut CRC
            'crc': frame[-1]
        }

    def build_frame(self, header: int, subheader: int, payload: bytes):
        """Construit une trame avec header/subheader et CRC."""
        length = len(payload)
        raw = bytearray([header, subheader, length]) + payload
        crc = sum(raw) & 0xFF
        raw.append(crc)
        return raw
