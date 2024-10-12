import numpy as np

data = np.array([1, 0, 1, 1, 0, 0, 1, 0], dtype=np.bool_) 

# Без явного указания bitorder
packed_bytes = np.packbits(data)  
print(packed_bytes.tobytes()) # Вывод: b'\x82\x00'

# С bitorder="little"
packed_bytes_little = np.packbits(data, bitorder="little")  
# print(packed_bytes_little.tobytes()) # Вывод: b'\x02\x80'