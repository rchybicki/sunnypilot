
from selfdrive.car.isotp_parallel_query import IsoTpParallelQuery


def enable_radar_tracks(CP, logcan, sendcan, debug=False):
  print("Try to enable radar tracks")
  rdr_fw = None
  for fw in CP.carFw:
    if fw.ecu == "fwdRadar":
      rdr_fw = fw
      break
  if rdr_fw is not None:
    print(f"Found fwdRadar: {rdr_fw.fwVersion}")
    for i in range(40):
      try:
        query = IsoTpParallelQuery(sendcan, logcan, 0, [rdr_fw.address], [b'\x10\x07'], [b'\x50\x07'], debug)
        for addr, dat in query.get_data(0.1).items(): # pylint: disable=unused-variable
          print(f"ecu write data by id try {i+1} ...")
          new_config = b"\x00\x00\x00\x01\x00\x01"
          dataId = b'\x01\x42'
          WRITE_DAT_REQUEST = b'\x2e'
          WRITE_DAT_RESPONSE = b'\x68'
          query = IsoTpParallelQuery(sendcan, logcan, 0, [rdr_fw.address], [WRITE_DAT_REQUEST+dataId+new_config], [WRITE_DAT_RESPONSE], debug)
          query.get_data(0)
          break
        print(f"Radar tracks enabled.")
        CP.radarUnavailable = False
        break
      except Exception as e:
        print(f"Failed {i}: {e}") 
  else:
    print("Failed to find fwdRadar")
  print("END Try to enable radar tracks")
