
from selfdrive.car.isotp_parallel_query import IsoTpParallelQuery


def enable_radar_tracks(CP, logcan, sendcan, debug=False):
  # START: Try to enable radar tracks
  print("Try to enable radar tracks")
  rdr_fw = None
  for fw in CP.carFw:
    if fw.ecu == "fwdRadar":
      rdr_fw = fw
      break
  print(f"Found fwdRadar: {rdr_fw.fwVersion}")
  try:
    for i in range(40):
      try:
        query = IsoTpParallelQuery(sendcan, logcan, 0, [rdr_fw.address], [b'\x10\x07'], [b'\x50\x07'],  debug = debug)
        for addr, dat in query.get_data(0.1).items(): # pylint: disable=unused-variable
          print("ecu write data by id ...")
          new_config = b"\x00\x00\x00\x01\x00\x01"
          dataId = b'\x01\x42'
          WRITE_DAT_REQUEST = b'\x2e'
          WRITE_DAT_RESPONSE = b'\x68'
          query = IsoTpParallelQuery(sendcan, logcan, 0, [rdr_fw.address], [WRITE_DAT_REQUEST+dataId+new_config], [WRITE_DAT_RESPONSE],  debug = debug)
          query.get_data(0)
          print(f"Try {i+1}")
          break
        break
      except Exception as e:
        print(f"Failed {i}: {e}") 
  except Exception as e:
    print("Failed to enable tracks" + str(e))
  print("END Try to enable radar tracks")
  # END try to enable radar tracks
