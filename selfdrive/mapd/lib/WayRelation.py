from selfdrive.mapd.lib.geo import DIRECTION, R, vectors, bearing_to_points, distance_to_points, point_on_line
from selfdrive.mapd.lib.osm import create_way
from common.conversions import Conversions as CV
from selfdrive.mapd.config import LANE_WIDTH
from common.basedir import BASEDIR
from datetime import datetime as dt
import numpy as np
import re
import json


_WAY_BBOX_PADING = 80. / R  # 80 mts of padding to bounding box. (expressed in radians)

with open(BASEDIR + "/selfdrive/mapd/lib/default_speeds_by_region.json", "rb") as f:
  DEFAULT_SPEEDS_BY_REGION = json.loads(f.read())

with open(BASEDIR + "/selfdrive/mapd/lib/default_speeds.json", "rb") as f:
  _COUNTRY_LIMITS = json.loads(f.read())


_WD = {
  'Mo': 0,
  'Tu': 1,
  'We': 2,
  'Th': 3,
  'Fr': 4,
  'Sa': 5,
  'Su': 6
}

_HIGHWAY_RANK = {
  'motorway': 0,
  'motorway_link': 1,
  'trunk': 10,
  'trunk_link': 11,
  'primary': 20,
  'primary_link': 21,
  'secondary': 30,
  'secondary_link': 31,
  'tertiary': 40,
  'tertiary_link': 41,
  'unclassified': 50,
  'residential': 60,
  'living_street': 61,
  'service': 62
}

speed_overrides_id = {

  1167942324: 40, #Przejazd kolejowy pomiędzy Domasławiem a Bielanami

  #Tyniec Domasławska
  1169316187: 13, #przed domem  
  1169294668: 45,
  35551085: 50,
  1169316186: 50,
  1167942316: 50,
  193054194: 50,
  1167942318: 50,
  1167942322: 50,
  1174857599: 50,
  1169295507: 60,

  #Tyniec Szczęśliwa
  1171211140: 5, #wjazd
  1171211141: 5, #wjazd
  133979428: 7,

  #Tyniec Świdnicka
  1167942312: 50,
  193054196: 50,
  43115448: 50,
  22926964: 50,
  193054197: 50,
  1169807993: 45,
  1169807992: 50,
  185542419: 55,

  #Wjazd na obwodnice z ronda tyniec w stronę miasta
  134429085: 65,
  360842016: 50, 
  1167942327: 120,

  #Zjazd z obwodnicy od strony miasta na rondo tyniec
  134429084: 60,
  360842012: 60,

  #Wjazd na obwodnice z Mokronosu w stronę Tyńca
  223324845: 59,
  1169806867: 60, 
  807593955: 70,
  111853026: 120,
  1176894307: 120,

  #Zjazd z obwodnicy od Tyńca w stronę Mokronosu
  111814380: 60, 
  223324842: 45,

  #Zjazd z obwodnicy od Kobierzyc w stronę Tyńca
  112228338: 70,
  112260512: 55,

  #Zjazd z A4 od strony Tyńca na Bielany
  322214383: 59, 
  330027681: 59, 
  272688751: 50,
  272688747: 50,
  249134773: 50,
  15800485: 59, 

  #Łacznik z obwodnicy na A4 w stronę Katowic
  111977997: 60,
  111977827: 60,
  248064919: 60,
  316438185: 65,
  316438184: 70,

  #pomiędzy rondami w strone bielan
  520977978: 65,
  185542421: 65,
  119181422: 65,
  118272068: 65,
  119181423: 65,
  904992498: 65,
  904992499: 65,
  206282955: 60,
  546342020: 60,
  546347506: 55,
  20555728: 55,

  #Mokronos Stawowa
  1171679035: 45,
  25118902: 45,
  1172221204: 60, 
  25118903: 60, 
  174140507: 60, 
  448924251: 60,
  
  #Mokronos Wrocławska
  1167946125: 50,
  1153652251: 50,
  1172221202: 50,
  782197059: 65, 
  1172221203: 65, 
  1153652252: 65, 

  #Wrocław Peronowa
  27037906: 25,
  186301632: 25,

  #Wrocław Wyścigowa,
  847529541: 55,
  164756672: 55,
  546364343: 55,
  307506268: 55,
  492667764: 55,
  513617269: 55,
  492667768: 55,
  492667772: 55,
  492667794: 55,
  546362994: 55,

  #Wrocław Aleja Karkonoska
  354270946: 60,
  28458096: 60,
  15779094: 55,
  19046874: 55,
  19046871: 60,
  481290473: 60,

  #Wrocław Grabiszyńska
  322072125: 40,
  15221185: 59,
  1056683531: 59,
  322072115: 59,
  1056664607: 59,
  235394664: 59,

  #Wrocław Wisniowa 
  28458756:  59, 
  28458757:  59,
  304347103: 59,
  371452465: 59,
  33782145: 59,
  22673803: 59,
  355999428: 59,
  189500455: 59,
  365799818: 59,
  304356445: 59,
  22673800: 59,
  830093165: 59,
  304356131: 59,
  321659947: 59,
  134516705: 59,
  32679156: 59,
  15804616: 59,

  #Wrocław Hallera
  22673799: 59, 
  321659949: 59,
  321659948: 59,
  28458758: 59,
  371236607: 59,
  304185996: 59,
  353537455: 59,
  28458759: 59,
  366609234: 59,
  366609235: 59,
  367371132: 59,
  304186366: 59,
  304186368: 59,
  353537456: 59,
  353537454: 59,
  353537452: 59,
  353537453: 59,
  361862364: 59,
  28460407: 59,
  28460404: 59,
  361862361: 59,
  304186826: 59,
  28460406: 59,
  353537457: 59,
  1159841445: 59,
  812988823: 59,
  1159841446: 59,
  32798115: 59,
  1046584787: 59,
  32798113: 59,
  15270680: 59,
  185948378: 59,
  
  #Wrocław Armii Krajowej
  355999427: 59, 
  16228087: 59,
  353537451: 59,
  186505212: 59,
  304348168: 59,
  28458082: 59,
  298102294: 59,
  186505208: 59,
  298102297: 59,
  298102300: 59,
  298102299: 59,

  #Wrocław Mokronoska 
  1154182997: 60, 
  18795673: 60,
  1154182996: 60,
  309925361: 60,
  1017147198: 60,
  695817935: 60,

  #Wrocław Zabrodzka
  25121542: 55,
  794401776: 55,

  #Wrocław Parkowa
  204008101: 50,
  309925360: 50,
  309925359: 50,
  223324841: 50,
  114136566: 50,
  448924250: 59,

  #Wrocław Ślężna
  1171175010: 25, #Hopka
  800318777: 50,
  618352949: 50,
  22673801: 59, 
  308361432: 59,
  800318778: 59,
  304182782: 59,
  304182783: 59,
  322266056: 59,
  304182929: 59,
  190721429: 59,
  546375967: 59,
  546375965: 59,
  546375966: 59,
  546375964: 59,
  190721424: 59,

  #Wrocław Kwiatkowskiego
  15921137: 60,
  15921138: 60,
  189327293: 59,
  695817934: 59,

  #Wrocław Tyniecka
  24983229: 59,

  #Wrocław Jeziorańskiego
  231316713: 65,
  307080161: 65,
  83410375: 65,
  370404883: 65,
  307080157: 60,

  #Wrocław Aleja Piastów
  15118987: 35,
  1153652257: 35,
  186153784: 35,
  443761216: 35,
  1154182994: 35,
  443761965: 35,
  1154182995: 35,
  437164390: 35,
  1172811811: 35,
  437164391: 35,
  423682103: 35,
  401158323: 35,
  185991363: 35,
  32798101: 35,
  443826986: 35,
  443826987: 35,
  1172811833: 35,
  1172811835: 35,
  1177217052: 35,
  1172811837: 35,
  1172811839: 35,
  1172811841: 35,
  1172811843: 35,
  1172811845: 35,
  1172811847: 35,
  1172811849: 35,
  1172811851: 35,
  1172811853: 35,
  450113197: 55,
  423682102: 55,
  640962322: 55,
  313926987: 55,
  783156556: 55,
  640961668: 55,
  373946571: 55,
  854827764: 55,
  854827765: 55,
  854827766: 55,
  177522534: 55
}

speed_overrides_id_forward = {
  #Mokronos Stawowa
  1171143758: 45,

  #Wrocław Parkowa
  204008100: 50,
  977768092: 50,
  
  #Tyniec Domasławska
  1167942314: 50
}

speed_overrides_id_backward = {
  #Tyniec Domasławska
  1167942314: 45
}

force_exp_mode_id = [
  #Tyniec początek Domasławskiej
  1168346112,
  1172963338,
  #Obwodnica przed zjazdem do Tyńca od miasta
  121496055,
  118272063,
  #Obwodnica przed zjazdem na Mokronos
  498067257,
  111853047,
  227138119,
  227138121,
  111853025,
  #Obwodnica przed zjazdem do Tyńca od Kobierzyc
  377273969,
  95288232,
  #A4 zjazd do miasta od strony Tyńca
  272587342,
  #S5 zjazd na Rawicz
  486463157,
  #Zabrodzie głowna
  25121881,
  189327315,
  189327400,
  #Wrocław Odkrywców
  362486306,
  #Wrocław Podróżnicza
  20308568
]

def is_osm_time_condition_active(condition_string):
  """
  Will indicate if a time condition for a restriction as described
  @ https://wiki.openstreetmap.org/wiki/Conditional_restrictions
  is active for the current date and time of day.
  """
  now = dt.now().astimezone()
  today = now.date()
  week_days = []

  # Look for days of week matched and validate if today matches criteria.
  dr = re.findall(r'(Mo|Tu|We|Th|Fr|Sa|Su[-,\s]*?)', condition_string)

  if len(dr) == 1:
    week_days = [_WD[dr[0]]]
  # If two or more matches condider it a range of days between 1st and 2nd element.
  elif len(dr) > 1:
    week_days = list(range(_WD[dr[0]], _WD[dr[1]] + 1))

  # If valid week days list is not empty and today day is not in the list, then the time-date range is not active.
  if len(week_days) > 0 and now.weekday() not in week_days:
    return False

  # Look for time ranges on the day. No time range, means all day
  tr = re.findall(r'([0-9]{1,2}:[0-9]{2})\s*?-\s*?([0-9]{1,2}:[0-9]{2})', condition_string)

  # if no time range but there were week days set, consider it active during the whole day
  if len(tr) == 0:
    return len(dr) > 0

  # Search among time ranges matched, one where now time belongs too. If found range is active.
  for times_tup in tr:
    times = list(map(lambda tt: dt.
                 combine(today, dt.strptime(tt, '%H:%M').time().replace(tzinfo=now.tzinfo)), times_tup))
    if now >= times[0] and now <= times[1]:
      return True

  return False


def speed_limit_value_for_limit_string(limit_string):
  # Look for matches of speed by default in kph, or in mph when explicitly noted.
  v = re.match(r'^\s*([0-9]{1,3})\s*?(mph)?\s*$', limit_string)
  if v is None:
    return None
  conv = CV.MPH_TO_MS if v[2] is not None and v[2] == "mph" else CV.KPH_TO_MS
  return conv * float(v[1])


def speed_limit_for_osm_tag_limit_string(limit_string):
  # https://wiki.openstreetmap.org/wiki/Key:maxspeed
  if limit_string is None:
    # When limit is set to 0. is considered not existing.
    return 0.

  # Attempt to parse limit as simple numeric value considering units.
  limit = speed_limit_value_for_limit_string(limit_string)
  if limit is not None:
    return limit

  # Look for matches of speed with country implicit values.
  v = re.match(r'^\s*([A-Z]{2}):([a-z_]+):?([0-9]{1,3})?(\s+)?(mph)?\s*', limit_string)
  if v is None:
    return 0.

  if v[2] == "zone" and v[3] is not None:
    conv = CV.MPH_TO_MS if v[5] is not None and v[5] == "mph" else CV.KPH_TO_MS
    limit = conv * float(v[3])
  elif f'{v[1]}:{v[2]}' in _COUNTRY_LIMITS:
    limit = speed_limit_value_for_limit_string(_COUNTRY_LIMITS[f'{v[1]}:{v[2]}'])

  return limit if limit is not None else 0.


def conditional_speed_limit_for_osm_tag_limit_string(limit_string):
  if limit_string is None:
    # When limit is set to 0. is considered not existing.
    return 0.

  # Look for matches of the `<restriction-value> @ (<condition>)` format
  v = re.match(r'^(.*)@\s*\((.*)\).*$', limit_string)
  if v is None:
    return 0.  # No valid format match

  value = speed_limit_for_osm_tag_limit_string(v[1])
  if value == 0.:
    return 0.  # Invalid speed limit value

  # Look for date-time conditions separated by semicolon
  v = re.findall(r'(?:;|^)([^;]*)', v[2])
  for datetime_condition in v:
    if is_osm_time_condition_active(datetime_condition):
      return value

  # If we get here, no current date-time condition is active.
  return 0.

def speed_limit_value_for_highway_type(areas, tags):
  max_speed = None
  try:
    geocode_country = ''
    geocode_region = ''
    for area in areas:
      if area.tags.get('admin_level', '') == "2":
        if area.tags.get('ISO3166-1:alpha2', '') != '':
          geocode_country = area.tags.get('ISO3166-1:alpha2', '')
      elif area.tags.get('admin_level', '') == "4":
        geocode_region = area.tags.get('name', '')
    country_rules = DEFAULT_SPEEDS_BY_REGION.get(geocode_country, {})
    country_defaults = country_rules.get('Default', [])
    for rule in country_defaults:
      rule_valid = all(
        tag_name in tags
        and tags[tag_name] == value
        for tag_name, value in rule['tags'].items()
      )
      if rule_valid:
        max_speed = rule['speed']
        break #stop searching country

    region_rules = country_rules.get(geocode_region, [])
    for rule in region_rules:
      rule_valid = all(
        tag_name in tags
        and tags[tag_name] == value
        for tag_name, value in rule['tags'].items()
      )
      if rule_valid:
        max_speed = rule['speed']
        break #stop searching region
  except KeyError as e:
    print(e)
  except TypeError as e:
    print(f"TypeError: {e} object is not iterable.")
  if max_speed is None:
    return 0
  v = re.match(r'^\s*([0-9]{1,3})\s*?(mph)?\s*$', str(max_speed))
  if v is None:
    return None
  conv = CV.MPH_TO_MS if v[2] is not None and v[2] == "mph" else CV.KPH_TO_MS
  return conv * float(v[1])


class WayRelation():
  """A class that represent the relationship of an OSM way and a given `location` and `bearing` of a driving vehicle.
  """
  def __init__(self, areas, way, parent=None):
    self.way = way
    self.areas = areas
    self.parent = parent
    self.parent_wr_id = parent.id if parent is not None else None  # For WRs created as splits of other WRs
    self.reset_location_variables()
    self.direction = DIRECTION.NONE
    self._speed_limit = None
    self._advisory_speed_limit = None
    self._one_way = way.tags.get("oneway")
    self.name = way.tags.get('name')
    self.ref = way.tags.get('ref')
    self.highway_type = way.tags.get("highway")
    self.highway_rank = _HIGHWAY_RANK.get(self.highway_type, 1000)
    try:
      self.lanes = int(way.tags.get('lanes'))
    except Exception:
      self.lanes = 2

    # Create numpy arrays with nodes data to support calculations.
    self._nodes_np = np.radians(np.array([[nd.lat, nd.lon] for nd in way.nodes], dtype=float))
    self._nodes_ids = np.array([nd.id for nd in way .nodes], dtype=int)

    # Get the vectors representation of the segments betwheen consecutive nodes. (N-1, 2)
    v = vectors(self._nodes_np) * R

    # Calculate the vector magnitudes (or distance) between nodes. (N-1)
    self._way_distances = np.linalg.norm(v, axis=1)

    # Calculate the bearing (from true north clockwise) for every section of the way (vectors between nodes). (N-1)
    self._way_bearings = np.arctan2(v[:, 0], v[:, 1])

    # Define bounding box to ease the process of locating a node in a way.
    # [[min_lat, min_lon], [max_lat, max_lon]]
    self.bbox = np.row_stack((np.amin(self._nodes_np, 0) - _WAY_BBOX_PADING,
                              np.amax(self._nodes_np, 0) + _WAY_BBOX_PADING))

    # Get the edge nodes ids.
    self.edge_nodes_ids = [way.nodes[0].id, way.nodes[-1].id]

  def __repr__(self):
    return f'(id: {self.id}, between {self.behind_idx} and {self.ahead_idx}, {self.direction}, active: {self.active})'

  def __eq__(self, other):
    if isinstance(other, WayRelation):
      return self.id == other.id
    return False

  def reset_location_variables(self):
    self.distance_to_node_ahead = 0.
    self.location_rad = None
    self.bearing_rad = None
    self.active = False
    self.diverting = False
    self.ahead_idx = None
    self.behind_idx = None
    self._active_bearing_delta = None
    self._distance_to_way = None

  @property
  def id(self):
    return self.way.id

  @property
  def road_name(self):
    if self.name is not None:
      return self.name
    return self.ref

  def update(self, location_rad, bearing_rad, location_stdev):
    """Will update and validate the associated way with a given `location_rad` and `bearing_rad`.
       Specifically it will find the nodes behind and ahead of the current location and bearing.
       If no proper fit to the way geometry, the way relation is marked as invalid.
    """
    self.reset_location_variables()

    # Ignore if location not in way bounding box
    if not self.is_location_in_bbox(location_rad):
      return

    # - Get the distance and bearings from location to all nodes. (N)
    bearings = bearing_to_points(location_rad, self._nodes_np)

    # - Get absolute bearing delta to current driving bearing. (N)
    delta = np.abs(bearing_rad - bearings)

    # - Nodes are ahead if the cosine of the delta is positive (N)
    is_ahead = np.cos(delta) >= 0.

    # - Possible locations on the way are those where adjacent nodes change from ahead to behind or vice-versa.
    possible_idxs = np.nonzero(np.diff(is_ahead))[0]

    # - when no possible locations found, then the location is not in this way.
    if len(possible_idxs) == 0:
      return

    projections = point_on_line(self._nodes_np[:-1], self._nodes_np[1:], location_rad)
    h = distance_to_points(location_rad, projections)

    # - Calculate the delta between driving bearing and way bearings. (N-1)
    bw_delta = self._way_bearings - bearing_rad

    # - The absolute value of the sin of `bw_delta` indicates how close the bearings match independent of direction.
    # We will use this value along the distance to the way to aid on way selection. (N-1)
    abs_sin_bw_delta = np.abs(np.sin(bw_delta))

    # - Get the delta to way bearing indicators and the distance to the way for the possible locations.
    abs_sin_bw_delta_possible = abs_sin_bw_delta[possible_idxs]
    h_possible = h[possible_idxs]

    # - Get the index where the distance to the way is minimum. That is the chosen location.
    min_h_possible_idx = np.argmin(h_possible)
    min_delta_idx = possible_idxs[min_h_possible_idx]
    projection = projections[min_delta_idx]

    # - If the distance to the way is over 4 standard deviations of the gps accuracy + the maximum road width
    # estimate, then we are way too far to stick to this way (i.e. we are not on this way anymore)
    # In theory the osm path is centered on the road which means half the road width would cover the whole road.
    # however, often times the osm path is not perfectly centered so we'll make the possible route more lenient by using
    # the full road width.
    road_width_estimate = self.lanes * LANE_WIDTH
    half_road_width_estimate = road_width_estimate / 2.
    if h_possible[min_h_possible_idx] > 4. * location_stdev + road_width_estimate:
      return

    # If the distance to the road is greater than 2 standard deviations of the gps accuracy + half the maximum road
    # width estimate + 1 lane width then we are most likely diverting from this route. Adding a lane width to give
    # leniency to not perfectly centered osm paths
    diverting = h_possible[min_h_possible_idx] > 2. * location_stdev + half_road_width_estimate + LANE_WIDTH

    # Populate location variables with result
    if is_ahead[min_delta_idx]:
      self.direction = DIRECTION.BACKWARD
      self.ahead_idx = min_delta_idx
      self.behind_idx = min_delta_idx + 1
    else:
      self.direction = DIRECTION.FORWARD
      self.ahead_idx = min_delta_idx + 1
      self.behind_idx = min_delta_idx

    self._distance_to_way = h[min_delta_idx]
    self._active_bearing_delta = abs_sin_bw_delta_possible[min_h_possible_idx]

    # find the distance to the next node by projecting our location onto the line and finding the delta between that
    # point and the next point on the route
    self.distance_to_node_ahead = distance_to_points(projection, np.array([self._nodes_np[self.ahead_idx]]))[0]
    self.active = True
    self.diverting = diverting
    self.location_rad = location_rad
    self.bearing_rad = bearing_rad
    self._speed_limit = None
    self._advisory_speed_limit = None
    self._force_exp_mode = None

  def update_direction_from_starting_node(self, start_node_id):
    self._speed_limit = None
    self._advisory_speed_limit = None
    self._force_exp_mode = None
    if self.edge_nodes_ids[0] == start_node_id:
      self.direction = DIRECTION.FORWARD
    elif self.edge_nodes_ids[-1] == start_node_id:
      self.direction = DIRECTION.BACKWARD
    else:
      self.direction = DIRECTION.NONE

  def is_location_in_bbox(self, location_rad):
    """Indicates if a given location is contained in the bounding box surrounding the way.
       self.bbox = [[min_lat, min_lon], [max_lat, max_lon]]
    """
    is_g = np.greater_equal(location_rad, self.bbox[0, :])
    is_l = np.less_equal(location_rad, self.bbox[1, :])

    return np.all(np.concatenate((is_g, is_l)))

  @property
  def speed_limit(self):
    if self._speed_limit is not None:
      return self._speed_limit

    # Get string from corresponding tag, consider conditional limits first.
    limit_string = self.way.tags.get("maxspeed:conditional")
    if limit_string is None:
      if self.direction == DIRECTION.FORWARD:
        limit_string = self.way.tags.get("maxspeed:forward:conditional")
      elif self.direction == DIRECTION.BACKWARD:
        limit_string = self.way.tags.get("maxspeed:backward:conditional")

    limit = conditional_speed_limit_for_osm_tag_limit_string(limit_string)

    # When no conditional limit set, attempt to get from regular speed limit tags.
    if limit == 0.:
      limit_string = self.way.tags.get("maxspeed")
      if limit_string is None:
        if self.direction == DIRECTION.FORWARD:
          limit_string = self.way.tags.get("maxspeed:forward")
        elif self.direction == DIRECTION.BACKWARD:
          limit_string = self.way.tags.get("maxspeed:backward")

      limit = speed_limit_for_osm_tag_limit_string(limit_string)

    if limit == 0.:
      limit = speed_limit_value_for_highway_type(self.areas, self.way.tags)

    self._speed_limit = limit
    return self._speed_limit
  
  
  @property
  def advisory_speed_limit(self):
    if self._advisory_speed_limit is not None:
      return self._advisory_speed_limit
    
    limit_string = None
    override = speed_overrides_id.get(self.way.id)
    if override is not None:
      limit_string = str(override)
    if limit_string is None:
      limit_string = self.way.tags.get("maxspeed:practical")
    if limit_string is None:
      if self.direction == DIRECTION.FORWARD:
        override = speed_overrides_id_forward.get(self.way.id)
        if override is not None:
          limit_string = str(override)
        if limit_string is None:
          limit_string = self.way.tags.get("maxspeed:practical:forward")
      elif self.direction == DIRECTION.BACKWARD:
        override = speed_overrides_id_backward.get(self.way.id)
        if override is not None:
          limit_string = str(override)
        if limit_string is None:
          limit_string = self.way.tags.get("maxspeed:practical:backward")
    if limit_string is None:
      limit_string = self.way.tags.get("maxspeed:advisory")
    limit = speed_limit_for_osm_tag_limit_string(limit_string)

    self._advisory_speed_limit = limit
    return self._advisory_speed_limit
  
  
  @property
  def force_exp_mode(self):
    if self._force_exp_mode is not None:
      return self._force_exp_mode
  
    self._force_exp_mode = self.way.id in force_exp_mode_id
    return self._force_exp_mode
  

  @property
  def active_bearing_delta(self):
    """Returns the sine of the delta between the current location bearing and the exact
       bearing of the portion of way we are currentluy located at.
    """
    return self._active_bearing_delta

  @property
  def is_one_way(self):
    return self._one_way in ['yes'] or self.highway_type in ["motorway"]

  @property
  def is_prohibited(self):
    # Direction must be defined to asses this property. Default to `True` if not.
    if self.direction == DIRECTION.NONE:
      return True
    return self.is_one_way and self.direction == DIRECTION.BACKWARD

  @property
  def distance_to_way(self):
    """Returns the perpendicular (i.e. minimum) distance between current location and the way
    """
    return self._distance_to_way

  @property
  def node_ahead(self):
    return self.way.nodes[self.ahead_idx] if self.ahead_idx is not None else None

  @property
  def last_node(self):
    """Returns the last node on the way considering the traveling direction
    """
    if self.direction == DIRECTION.FORWARD:
      return self.way.nodes[-1]
    if self.direction == DIRECTION.BACKWARD:
      return self.way.nodes[0]
    return None

  @property
  def last_node_coordinates(self):
    """Returns the coordinates for the last node on the way considering the traveling direction. (in radians)
    """
    if self.direction == DIRECTION.FORWARD:
      return self._nodes_np[-1]
    if self.direction == DIRECTION.BACKWARD:
      return self._nodes_np[0]
    return None

  def node_before_edge_coordinates(self, node_id):
    """Returns the coordinates of the node before the edge node identifeid with `node_id`. (in radians)
    """
    if self.edge_nodes_ids[0] == node_id:
      return self._nodes_np[1]

    if self.edge_nodes_ids[-1] == node_id:
      return self._nodes_np[-2]

    return np.array([0., 0.])

  def split(self, node_id, way_ids=None):
    """ Returns and array with the way relations resulting from splitting the current way relation at node_id
    """
    idxs = np.nonzero(self._nodes_ids == node_id)[0]
    if len(idxs) == 0:
      return []

    idx = idxs[0]
    if idx == 0 or idx == len(self._nodes_ids) - 1:
      return [self]

    if not isinstance(way_ids, list):
      way_ids = [-1, -2]  # Default id values.

    ways = [create_way(way_ids[0], node_ids=self._nodes_ids[:idx + 1], from_way=self.way),
            create_way(way_ids[1], node_ids=self._nodes_ids[idx:], from_way=self.way)]
    return [WayRelation(self.areas, way, parent=self) for way in ways]
