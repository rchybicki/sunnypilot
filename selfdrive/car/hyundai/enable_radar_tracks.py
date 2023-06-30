import time
from selfdrive.car.isotp_parallel_query import IsoTpParallelQuery

def _fetch_rdr_fw(CP):
    return next((fw for fw in CP.carFw if fw.ecu == "fwdRadar"), None)

def _verify_radar(rdr_fw, logcan, sendcan, debug):
    return IsoTpParallelQuery(sendcan, logcan, 0, [rdr_fw.address], [b'\x10\x07'], [b'\x50\x07'], debug=debug)

def _radar_tracks_enable_query(rdr_fw, logcan, sendcan, debug):
    
        new_config = b"\x00\x00\x00\x01\x00\x01"
        dataId = b'\x01\x42'
        WRITE_DAT_REQUEST = b'\x2e'
        WRITE_DAT_RESPONSE = b'\x68'
        query = IsoTpParallelQuery(sendcan, logcan, 0, [rdr_fw.address],
                                   [WRITE_DAT_REQUEST+dataId+new_config], [WRITE_DAT_RESPONSE], debug=debug)
        query.get_data(0)

def _enable_radar_tracks(rdr_fw, logcan, sendcan, debug):
    try:
        for retries in range(10):
            query = _verify_radar(rdr_fw, logcan, sendcan, debug)
            print(f"radar_tracks: ecu write data by id try {retries+1} ...")
            if query.get_data(0.1):  # Check if any data is returned
              _radar_tracks_enable_query(rdr_fw, logcan, sendcan, debug)
            break
    except Exception as e:
        time.sleep(3)
        print(f"radar_tracks: Failed {retries} try: {e}")

def enable_radar_tracks(CP, logcan, sendcan, debug=False):
    print("radar_tracks: Try to enable radar tracks")

    rdr_fw = _fetch_rdr_fw(CP)

    if rdr_fw is not None:
        print(f"radar_tracks: Found fwdRadar: {rdr_fw.fwVersion}")
        _enable_radar_tracks(rdr_fw, logcan, sendcan, debug)
        print("radar_tracks: Radar tracks enabled.")
    else:
        print("radar_tracks: Failed to find fwdRadar")