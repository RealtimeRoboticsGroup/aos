from gi.repository import Gtk
from collections import namedtuple

window = Gtk.Window()
screen = window.get_screen()

#Set screen size for rest of program.
SCREEN_SIZE = screen.get_height() / 1.5

# Placeholder value
ROBOT_SIDE_TO_BALL_CENTER = 0.15
BALL_RADIUS = 0.165

# Placeholder value
ROBOT_SIDE_TO_HATCH_PANEL = 0.1
HATCH_PANEL_WIDTH = 0.4826

# field_id is either just a file prefix for a .png in field_images/ or is a
# full path preceded by // specifying a location relative to the root of the
# repository.
FieldType = namedtuple(
    'Field', ['name', 'tags', 'year', 'width', 'length', 'robot', 'field_id'])
RobotType = namedtuple("Robot", ['width', 'length'])

GALACTIC_SEARCH = "Galactic Search"
ARED = "A Red"
BRED = "B Red"
ABLUE = "A Blue"
BBLUE = "B Blue"
AUTONAV = "AutoNav"
BOUNCE = "Bounce"
SLALOM = "Slalom"
BARREL = "Barrel"

Robot2019 = RobotType(width=0.65, length=0.8)
Robot2020 = RobotType(width=0.8128, length=0.8636)  # 32 in x 34 in
Robot2021 = Robot2020
Robot2022 = RobotType(width=0.8763, length=0.96647)
Robot2023 = RobotType(width=0.6061, length=0.77581)
Robot2024 = RobotType(width=0.85979, length=0.9906)  # 33.85 in x 39.0 in

FIELDS = {
    "2019 Field":
    FieldType("2019 Field",
              tags=[],
              year=2019,
              width=8.258302,
              length=8.258302,
              robot=Robot2019,
              field_id="2019"),
    "2020 Field":
    FieldType("2020 Field",
              tags=[],
              year=2020,
              width=15.98295,
              length=8.21055,
              robot=Robot2020,
              field_id="2020"),
    "2021 Galactic Search BRed":
    FieldType("2021 Galactic Search BRed",
              tags=[GALACTIC_SEARCH, BRED],
              year=2021,
              width=9.144,
              length=4.572,
              robot=Robot2021,
              field_id="red_b"),
    "2021 Galactic Search ARed":
    FieldType("2021 Galactic Search ARed",
              tags=[GALACTIC_SEARCH, ARED],
              year=2021,
              width=9.144,
              length=4.572,
              robot=Robot2021,
              field_id="red_a"),
    "2021 Galactic Search BBlue":
    FieldType("2021 Galactic Search BBlue",
              tags=[GALACTIC_SEARCH, BBLUE],
              year=2021,
              width=9.144,
              length=4.572,
              robot=Robot2021,
              field_id="blue_b"),
    "2021 Galactic Search ABlue":
    FieldType("2021 Galactic Search ABlue",
              tags=[GALACTIC_SEARCH, ABLUE],
              year=2021,
              width=9.144,
              length=4.572,
              robot=Robot2021,
              field_id="blue_a"),
    "2021 AutoNav Barrel":
    FieldType("2021 AutoNav Barrel",
              tags=[AUTONAV, BARREL],
              year=2021,
              width=9.144,
              length=4.572,
              robot=Robot2021,
              field_id="autonav_barrel"),
    "2021 AutoNav Slalom":
    FieldType("2021 AutoNav Slalom",
              tags=[AUTONAV, SLALOM],
              year=2021,
              width=9.144,
              length=4.572,
              robot=Robot2021,
              field_id="autonav_slalom"),
    "2021 AutoNav Bounce":
    FieldType("2021 AutoNav Bounce",
              tags=[AUTONAV, BOUNCE],
              year=2021,
              width=9.144,
              length=4.572,
              robot=Robot2021,
              field_id="autonav_bounce"),
    "2022 Field":
    FieldType("2022 Field",
              tags=[],
              year=2022,
              width=16.4592,
              length=8.2296,
              robot=Robot2022,
              field_id="2022"),
    "2023 Field":
    FieldType("2023 Field",
              tags=[],
              year=2023,
              width=16.59255,
              length=8.10895,
              robot=Robot2023,
              field_id="//third_party/y2023/field/2023.png"),
    "2024 Field":
    FieldType("2024 Field",
              tags=[],
              year=2024,
              width=16.54175,
              length=8.21055,
              robot=Robot2024,
              field_id="//third_party/y2024/field/2024.png"),
}

FIELD = FIELDS["2024 Field"]


def get_json_folder(field):
    return "frc/control_loops/python/spline_jsons"


def inToM(i):
    return (i * 0.0254)
