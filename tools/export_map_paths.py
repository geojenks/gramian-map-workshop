"""The records as tip paths for the 3D map page: one JSON file the page loads.

    python tools/export_map_paths.py [--drafts] [--out outputs/map_paths] [--tag v2]
    python tools/export_map_paths.py --site

Default: writes outputs/map_paths/map_paths[_with_drafts][_<tag>].json and
refuses to overwrite it (--tag adds a suffix so a new run gets a new name).
--drafts also reads the draft records in data/draft_records/ and marks them
"draft": true, carrying their flags.

--site writes site/data/map_paths.json instead, always with the drafts, and
REPLACES that file: it is a tracked file that this script regenerates, so
git keeps its history. --out and --tag are ignored with --site.

Each actuator is drawn as one or more legs. A leg is a path of tip motion as
one input rises, read straight from the record's free tip curve
(free.u, free.tip_displacement_m), in the record's own frame (x is the
base-to-tip axis). The stored displacement is used as it is: it is measured
from rest, and nothing is subtracted from it.

  single input   one leg: every stored level, labelled with the input's
                 name, in order of the size of the input (for every record
                 we hold, that is the stored order).
  several inputs one leg per input channel and per setting of all the other
                 channels: the rows with that setting, ordered by the size
                 of the channel's input; kept only if it has at least 3
                 rows.

Per level each leg carries axial travel (x), sideways travel (y, z), bending
travel (the length of the sideways part), tip twist (the x part of
free.tip_rotvec where the record holds it, otherwise null for the whole leg:
not measured, never zero) and the input share. The input share of any set of
inputs is the largest, over the input channels, of the size of that
channel's input divided by the largest size of input that channel takes
anywhere in the record's free rows. For a single-input record that is the
size of the input over the largest size in the leg. A dotted lead-in from
the origin is asked for when the first drawn row is off the origin and every
other input channel is at zero.

An actuator with exactly two input channels gets a "surface" when
data/map_two_input_rules.json declares how its two inputs combine, each
channel has a leg with the other channel at exactly 0, and both of those
legs start at zero input. The surface is a grid of computed tip positions
for every pair of those single-input levels, built from the two
single-input paths by the declared rule (bending is always the length of
the computed sideways part, twist the x part of the computed rotation
vector):

  series   two segments end to end, A nearer the base and B beyond it: the
           far segment's motion is carried on the near segment's tip, so
           displacement = d_A + R_A applied to d_B, and rotation = R_A
           composed with R_B (B's rotation first, then A's). Needs the
           record's tip rotation; a series record without it is an error.
  sum      the two tip displacements added, x, y and z separately, and the
           two tip rotation vectors added.

It is computed, not measured. Every free row where both channels are
non-zero becomes a test point that compares the stored row with the surface
at its inputs (the single-input legs are interpolated linearly where an
input is not one of their levels; an input outside a leg's range is an
error, never extrapolated or clamped). Under the series rule each test point
also carries the plain sum, for comparison. A two-input actuator with no
declaration, and every other actuator, carries "surface": null.

data/map_extra_check_points.json may name, for a two-input record, a CSV of
further simulated tip positions with both inputs on. Those are never used to
draw the surface and are kept apart from the record's own test points: the
surface carries them as "extra_checks" (null without such a file), each
point compared with the surface at its inputs in tip position only, since
the file holds no tip rotation. Only rows the file marks as simulated are
used; rows it marks otherwise (filled in by a calculation) are left out and
counted. The file's axes are not trusted: every simulated row that the
record also holds must give the record's own tip displacement to within
EXTRA_MATCH_M, or the export stops.

Each actuator also carries its body length: the record's own
metadata.length_m where that is a positive number, otherwise an estimate
from data/map_lengths.json (which must say where its number comes from and
may never stand in for a length the record already holds), otherwise none.
With a length L, each level also carries the tip travel as a share of L and
an estimate of the body's shape from the tip position alone: the body is
taken as one even arc from the base (the rest tip at (L, 0, 0)) to the tip,
which gives a bend angle, its parts toward y and toward z, and the stretch
of the arc over L. The estimate is exact for such a body and wrong for a
body that bends in only part of its length; where the tip is closer to the
base than CHORD_FLOOR of L there is no estimate at that level. Without a
length all of these are null for the whole leg: not measured, never zero.
Each leg also says how far the bend and stretch move when L is changed by
LENGTH_TOLERANCE either way, and where the record holds the tip rotation
the actuator carries "bend_check": the tilt of the tip measured from that
rotation beside the estimated bend, at each leg's top level.

Limits the records do not carry come from data/map_caveats.json ("draw":
false leaves a record out with the reason given; "caveat" is shown beside
it). Actuators placed by hand, not from a record, come from
data/map_hand_placed.json; each must say where its placement came from
("basis"), and none comes from, or goes into, data/records.

A task is drawn as a cone on the three tip-travel axes (axial, y, z): its
tip at the origin, about the direction the task needs the tip to move in.
The cones are declared in data/map_task_cones.json: the task file, the
placement of the finger the direction is worked out for, the half-angle
with where it comes from, and the least travel (null where the task file
states none; the cone then has no floor). The needed direction is the one
demos/backwards_lookup.py uses, worked out per record, so it differs a
little from record to record; each record is tested against its own, and
the document also gives the one direction the cone is drawn about. The
test is on the tip position from rest at each measured level, which is not
the lookup's test (the lookup takes the steps between levels over part of
the input range).

Where a cone declares "stroke" (the wave's commanded stroke angle, with
where it is read from), the cone gets a far end: "stroke.travel_m" is the
tip travel for one stroke, the stroke angle in radians times the ball
radius times the cosine of the contact-ring altitude (both from the task
file), the inverse of Arrangement.turn_per_stroke_deg. Each actuator's
task entry then carries "stroke": per leg, the travel along its needed
direction at each level ("along_m"), the size of the travel at right angles
to that direction at each level ("across_m": a size, never negative, worked
out from the same tip position and direction, so that the two together give
the angle to the needed direction), whether it reaches the far end, the
input at which it first does (in a straight line between the two logged
levels either side), and whether every level up to that point is inside
the cone. The stroke is taken to start with the finger at rest touching
the ball; the straight cone with a far end stands in for the tip's real
target, an arc about the turn axis.

Each actuator also carries "force": what its blocked block holds with the
tip held, at that block's own input levels, with the record's own method
text; see FORCE_NOTE.

Where a cone that has a stroke also declares "return_clear", each task
entry carries a second check under that key: after the stroke the finger
must break contact and come back to its start clear of the object. The
stroke path is the first leg that passes the first check (inside the angle
all the way to one stroke), taken, as the stroke's own basis says, to run
in contact from rest. At RETURN_STATIONS evenly spaced travels along the
stroke, up to one stroke, every tip position the export holds at that
travel is read off (each leg, and each grid line of a two-input surface,
in a straight line between logged levels) and its travel along the contact
normal is compared with the stroke path's there. The contact normal points
into the object (gramian_workshop/task.py), so less travel along it is
further out. "outward_reach_m" is how much further out of the object than
the stroke path the tip can sit at each station, "inward_reach_m" how much
further in, and the check passes when the least outward reach over the
stations is at least the declared "drawn_depth_m". That depth is a display
choice, since the task file states no clearance; it also sets how deep the
page draws the band the return must run in. Distance from the object's
curved surface is not used: the stroke is drawn straight, and a straight
path leaves a ball by geometry alone. A record that does not pass the
first check has no stroke path, and its entry says so.

A cone's "return_clear" may also declare "stroke_from": "own_start" (with
"stroke_from_basis" saying why). The stroke path may then be any line of
tip positions the export holds, swept upwards in its input: a leg, or a
grid line of a two-input surface, which is one input swept with the other
held. It is judged from its own first point, not from rest: it must reach
one stroke's travel along the needed direction from there, within the
cone's half-angle of that direction all the way. The return is one other
such line (or the same one). At each station the gap is how much further
out of the object the return line sits than the stroke line; where a line
is at a station's travel more than once, its position furthest into the
object is the one taken. The pair of lines with the greatest least gap is
reported ("stroke_line", "return_line", with the points of each at the
start and at each station), and the check passes when that least gap is at
least "drawn_depth_m". Getting from one line to the other, at the two ends
of the stroke, is a change of the held input and is not checked. Without
"stroke_from" nothing above changes.

Where a cone declares "push_basis" and "push_threshold_ratio", the cone
also carries "push": the push each finger must make for the hand to hold
the task's object, worked out by the repo's hold solve
(readout.soft_hold with a margin) from the task file and the cone's
placement, with a finger of unit force; the needed push is 1 over the
smallest margin over the contact sets the duty schedule makes. Each
actuator's task entry then carries "push": the same solve with the
record's own push direction and unit force, and the push the record can
make along that direction at each held-tip level, compared with it. The
record's own force is never divided by its margin, because the margin
search stops at a cap.

data/map_force_withheld.json lists records whose force with the tip held
is left off the map, each with its reason and evidence pinning the record
file by sha256 (the export stops if that file no longer matches). A listed
record is treated as one that holds no force with the tip held: "force" is
null, "has_force" is false, "no_force_data" is among its flags, each task
entry's "push" is null (not tested, as for a record with no force), and
its scaled force is null. Its paths, and its placement on each task, are
read from the record as before. Each actuator carries "force_withheld"
(the reason, or null), and the document carries "force_withheld": the
sidecar's entries as checked, by record id. An id that no record or draft
record has is refused.

The document ends with "scaled": quantities with size and material taken
out, kept apart so nothing above changes. For one geometry the shape
depends only on the input over the body's stiffness, and at the same shape
the force grows with E L squared (E the body's Young's modulus, L the body
length). The body modulus is the record's own
metadata.materials.young_modulus_body where set (the McKibben records also
give the limiter's, and its ratio to the body's), otherwise the value
declared in data/map_stiffness.json, which pins the scene file it was read
from by commit, path, sha256, symbol and line, and says why that value is
the one the run used; it is never declared for a record that holds its own.
A record with neither has no modulus, and its scaled input (where the form
needs one) and scaled force are null with the reason.
data/map_pressure_scale.json holds corrections to a simulated pressure: each
multiplies the
pressure by 1 / dt for the records it lists and no others (chosen from
their provenance: a simulation in parametricGripper.py with an input in
Pa), because the scene's pressure constraint takes its value as an impulse
over a time step. That correction has not been ruled on; deleting its entry
changes only the scaled input. "scaled" holds:

  note             : SCALED_NOTE.
  quantities       : SCALED_QUANTITIES, each with its "definition":
                     input_scaled (the leg's own input as a pure number, in
                     one of three forms: a pressure over E, after any
                     correction; a cable tension in N over E L squared; a
                     cable pull in m over L, which needs no stiffness and
                     takes no correction), side_y_share and side_z_share
                     (tip travel along y and z over L). Axial and sideways
                     travel over L are axial_share and bending_share above
                     and are not repeated.
  force_quantities : SCALED_FORCE_QUANTITIES, each with its "definition":
                     size_scaled, axial_scaled and sideways_scaled (the
                     held-tip force size, its axial part, and the size of its
                     sideways part, over E L squared) and input_scaled (the
                     held-tip levels' inputs, one per channel, by the path's
                     rule).
  pressure_scale   : each correction as written, with its factor.
  actuators        : per drawn actuator id: "stiffness" (stiffness_of), and
                     per leg, in the actuator's leg order, "values" in
                     SCALED_KEYS, "input_scaled_form" (one of FORM_PRESSURE,
                     FORM_TENSION, FORM_PULL), "input_scaled_definition" (the
                     leg's rule in words, naming the channel, the correction
                     and E or L used, and on a leg with another input held
                     above zero, that only the leg's own input is scaled) and
                     "reasons" (why a null value is null); and "force" (null
                     without a held-tip force).

The file holds no timestamps and no git commit, so the same inputs always
give the same bytes. Floats are written at full precision. Records are read,
never edited. Simulated magnitudes are comparative, not predictive.
"""
import argparse
import csv
import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "demos"))
sys.path.insert(0, os.path.join(ROOT, "tools"))

from gramian_workshop.finger import Finger, load_manifest, load_record  # noqa: E402
from gramian_workshop.lookup import directions, free_curve  # noqa: E402
from gramian_workshop.task import load_task  # noqa: E402
from gramian_workshop.arrange import Arrangement  # noqa: E402
from gramian_workshop.readout import soft_hold  # noqa: E402
from gramian_workshop.turn import schedule  # noqa: E402
from gramian_workshop.vendor.capability_core import SCALE_MAX  # noqa: E402
from _provenance import sha256  # noqa: E402

MADE_BY = "tools/export_map_paths.py"
NOTE = ("Paths are values read from the records. A surface is computed from "
        "two paths and is not a measurement. Simulated magnitudes are for "
        "comparing actuators, not forecasts of hardware. Lengths in metres, "
        "angles in radians.")
CAVEATS = os.path.join(ROOT, "data", "map_caveats.json")
TWO_INPUT_RULES = os.path.join(ROOT, "data", "map_two_input_rules.json")
RULES = ("series", "sum")
EXTRA_CHECKS = os.path.join(ROOT, "data", "map_extra_check_points.json")
EXTRA_KEYS = ("file", "sha256", "inputs", "position_mm", "to_record_axes",
              "use", "why")
EXTRA_MATCH_M = 1e-8   # a shared row must give the record's tip displacement this closely
EXTRA_MIN_SHARED = 3   # shared rows needed with each input on, to trust the axes
RETURN_STATIONS = 16   # travels along the stroke at which the second check is read
RETURN_KEYS = ("basis", "drawn_depth_m", "drawn_depth_basis")
RETURN_OPTIONAL_KEYS = ("stroke_from", "stroke_from_basis")  # both or neither
STROKE_FROM_OWN_START = "own_start"
NO_STROKE_LINE = ("No line of its tip positions reaches one stroke from "
                  "its own start inside the cone's angle, so it has no "
                  "stroke path to come back along.")
NO_STROKE_PATH = ("It does not pass the first check, so it has no stroke "
                  "path to come back along.")
HAND_PLACED = os.path.join(ROOT, "data", "map_hand_placed.json")
LENGTHS = os.path.join(ROOT, "data", "map_lengths.json")
TASK_CONES = os.path.join(ROOT, "data", "map_task_cones.json")
STIFFNESS = os.path.join(ROOT, "data", "map_stiffness.json")
PRESSURE_SCALE = os.path.join(ROOT, "data", "map_pressure_scale.json")
FORCE_WITHHELD = os.path.join(ROOT, "data", "map_force_withheld.json")
NAMES = os.path.join(ROOT, "data", "map_names.json")
THUMBNAILS = os.path.join(ROOT, "data", "map_thumbnails.json")
SITE_DIR = os.path.join(ROOT, "site")
# who a display name is from (data/map_names.json)
NAME_FROM = ("authors", "field", "description")
NAME_MAX = 48                   # characters: a name is a label on the map
DOI_LINK = "https://doi.org/"
THUMBNAIL_KINDS = ("mesh", "schematic")
THUMBNAIL_PIN_KEYS = ("repo", "commit", "path", "sha256")
THUMBNAIL_PARTS = ("body", "braid", "tendon", "cables")
MESH_BASIS = ("Real geometry: the rest shape, drawn from the mesh the "
              "simulation ran on.")
THUMBNAIL_NOTE = (
    "The small picture of each actuator. Kind \"mesh\" is real geometry: "
    "the rest shape, drawn from the mesh the simulation ran on. Kind "
    "\"schematic\" is a plain diagram of the type of actuator, drawn by "
    "us from its description; it is not the geometry of the published "
    "device. Pictures are not to a common scale. An actuator with no "
    "picture has \"thumbnail\" null.")
# what a pinned source must name: enough to fetch the file with git show and
# check it byte for byte, and the symbol and line the value is read from
PIN_KEYS = ("repo", "commit", "path", "sha256", "symbol", "line")
# the simulation scene whose pressure input the pressure correction is for
SCENE_NAME = "parametricGripper.py"
CONE_AXES = ["axial_m", "side_y_m", "side_z_m"]
# The most --stroke-deg the wave script accepts (it raises above this).
STROKE_MAX_DEG = 50.0
STROKE_SOURCE_KEYS = ("commit", "path", "sha256", "key", "flag")
STROKE_END_INPUT_IS = "interpolated between logged levels"
# A leg whose first logged level is already at or beyond the end has no
# level below it to interpolate from; its end input is that level's own.
STROKE_FIRST_LEVEL_IS = "already at or beyond the end at the first logged level"
FORCE_NOTE = (
    "Force with the tip held, read from the record's blocked block, at the "
    "input levels that block holds and no others. Most simulated records "
    "hold the tip at its rest position; some published ones hold it "
    "elsewhere, and each actuator's own method text says how. So this is "
    "the force at one held pose, not along the tip path. Axial force is "
    "signed as stored, along the base-to-tip axis. Where the record holds "
    "the whole force vector its direction is known; where it holds only "
    "the size of the sideways part, the direction of that part about the "
    "axis is not known. A part the record does not hold is null: not "
    "measured, not zero. Moments are about the inlet centroid. The "
    "simulated forces were read before the body had fully settled, so "
    "they are comparative, not predictive.")
SITE_FILE = os.path.join(ROOT, "site", "data", "map_paths.json")
# the stored backwards lookup: its "separation.compared" list is the set of
# clean records the push range over passers is taken across
LOOKUP_OUT = os.path.join(ROOT, "outputs", "backwards_lookup.json")
PUSH_NS = (3, 4, 5, 6)          # finger counts the needed push is also given for
NO_MARGIN = ("no_contact", "direction", "friction", "cone_edge")
MULTI_INPUT_BOTH = (
    "Not tested: the held-tip levels set both inputs together, and the "
    "free-path legs are other input pairs, so no level is measured both "
    "ways.")
MIN_LEG_LEVELS = 3     # a leg needs at least this many rows
LENGTH_TOLERANCE = 0.05  # the length is changed by this share either way
# no shape estimate where the tip is closer to the base than this share of
# the length: there the tip's direction no longer says how far the body has
# bent (an even arc that does not stretch reaches it at about 270 degrees)
CHORD_FLOOR = 0.3

TRAVEL = "Tip travel"
INPUT = "Input"
SHARE_OF_LENGTH = "Tip travel as a share of length"
BODY_SHAPE = "Body shape, estimated from the tip position"
ARC_ASSUMPTION = (
    "It assumes the body is one even arc from the base to the tip, with the "
    "tip at rest a body length along the x axis. It is exact for such a "
    "body, reads zero bend for a body that only extends, and is wrong for a "
    "body that bends in only part of its length (on the two tandem twist "
    "drafts, which bend in two parts, it does not match the tilt of the tip "
    "measured from the stored tip rotation). The bend lies between 0 and "
    "360 degrees, so a body curling past a full turn is not represented. No "
    f"estimate where the tip is closer to the base than {CHORD_FLOOR:.0%} "
    "of the body length, because there the tip's direction no longer says "
    "how far the body has bent. Null for the whole leg where the actuator "
    "has no length: not measured, not zero.")

QUANTITIES = [
    {"key": "axial_m", "label": "Axial tip travel", "unit": "m",
     "group": TRAVEL, "estimate": False,
     "definition":("Tip displacement from rest along the record's x axis, "
                    "the base-to-tip axis, as stored in "
                    "free.tip_displacement_m. Positive is the tip moving "
                    "away from the base.")},
    {"key": "bending_m", "label": "Bending tip travel", "unit": "m",
     "group": TRAVEL, "estimate": False,
     "definition": ("Length of the sideways part of the stored tip "
                    "displacement: the square root of (y squared plus z "
                    "squared). Measured from rest, like the stored "
                    "displacement, not from the first measured level.")},
    {"key": "twist_rad", "label": "Tip twist", "unit": "rad",
     "group": TRAVEL, "estimate": False,
     "definition": ("Rotation of the tip about the record's x axis: the x "
                    "component of the tip rotation vector stored in "
                    "free.tip_rotvec, at each level. Null for the whole leg "
                    "where the record holds no tip rotation: not measured, "
                    "not zero.")},
    {"key": "side_y_m", "label": "Sideways travel, y", "unit": "m",
     "group": TRAVEL, "estimate": False,
     "definition": ("Tip displacement from rest along the record's y axis, "
                    "as stored in free.tip_displacement_m.")},
    {"key": "side_z_m", "label": "Sideways travel, z", "unit": "m",
     "group": TRAVEL, "estimate": False,
     "definition": ("Tip displacement from rest along the record's z axis, "
                    "as stored in free.tip_displacement_m.")},
    {"key": "input_share", "label": "Share of input range", "unit": "",
     "group": INPUT, "estimate": False,
     "definition": ("For each input channel, the size of its input divided "
                    "by the largest size of input that channel takes "
                    "anywhere in the record's free rows; the share is the "
                    "largest of these over the channels. Every value lies "
                    "between 0 and 1. For a single-input record it is the "
                    "size of the input over the largest size in the leg: "
                    "it rises along the leg, starts at 0 where the first "
                    "input is zero and ends at exactly 1. On a leg with "
                    "another channel held above zero it starts at that "
                    "channel's share (0.5 for a chamber held at half its "
                    "largest pressure) and stays there until the leg's own "
                    "input passes it. Works the same way for vacuum "
                    "(negative) inputs.")},
    {"key": "bend_rad", "label": "Bend angle (estimate)", "unit": "rad",
     "group": BODY_SHAPE, "estimate": True,
     "definition": ("Estimated from the tip position and the body length: "
                    "twice the angle at the base between the x axis and the "
                    "straight line from the base to the tip. "
                    + ARC_ASSUMPTION)},
    {"key": "stretch_share", "label": "Stretch (estimate)", "unit": "",
     "group": BODY_SHAPE, "estimate": True,
     "definition": ("Estimated from the tip position and the body length: "
                    "the length of the even arc from the base to the tip, "
                    "divided by the body length, less 1. Positive is the "
                    "body longer than at rest. It depends strongly on the "
                    "body length for a body that bends a lot; each leg "
                    "says how far it moves when the length is changed by "
                    f"{LENGTH_TOLERANCE:.0%} either way. " + ARC_ASSUMPTION)},
    {"key": "bend_y_rad", "label": "Bend toward y (estimate)", "unit": "rad",
     "group": BODY_SHAPE, "estimate": True,
     "definition": ("The estimated bend angle times the share of the "
                    "sideways tip travel that lies along the record's y "
                    "axis (y travel over sideways travel); 0 where the tip "
                    "has not moved sideways. " + ARC_ASSUMPTION)},
    {"key": "bend_z_rad", "label": "Bend toward z (estimate)", "unit": "rad",
     "group": BODY_SHAPE, "estimate": True,
     "definition": ("The estimated bend angle times the share of the "
                    "sideways tip travel that lies along the record's z "
                    "axis (z travel over sideways travel); 0 where the tip "
                    "has not moved sideways. " + ARC_ASSUMPTION)},
    {"key": "axial_share", "label": "Axial tip travel ÷ length", "unit": "",
     "group": SHARE_OF_LENGTH, "estimate": False,
     "definition": ("Axial tip travel divided by the body length. A body "
                    "that bends also brings its tip back toward the base, "
                    "so a negative value does not by itself mean the body "
                    "got shorter. Null for the whole leg where the actuator "
                    "has no length: not measured, not zero.")},
    {"key": "bending_share", "label": "Sideways tip travel ÷ length",
     "unit": "", "group": SHARE_OF_LENGTH, "estimate": False,
     "definition": ("Bending tip travel (the length of the sideways part of "
                    "the tip displacement) divided by the body length. Null "
                    "for the whole leg where the actuator has no length: "
                    "not measured, not zero.")},
]
VALUE_KEYS = [q["key"] for q in QUANTITIES]
# the quantities that need the body length, and the estimates among them
LENGTH_KEYS = ["bend_rad", "stretch_share", "bend_y_rad", "bend_z_rad",
               "axial_share", "bending_share"]
ESTIMATE_KEYS = ["bend_rad", "stretch_share", "bend_y_rad", "bend_z_rad"]
SENSITIVITY_KEYS = ["bend_rad", "stretch_share"]
ACTUATOR_KEYS = ["id", "name", "physics", "source_kind", "draft",
                 "hand_placed", "has_force", "flags", "caveat",
                 "record_file", "record_sha256", "legs", "surface",
                 "length", "bend_check", "tasks", "force", "force_withheld",
                 "display_name", "name_detail", "name_from", "name_source",
                 "reference", "thumbnail"]
# the keys that come from data/map_names.json and data/map_thumbnails.json
SHOWN_KEYS = ("display_name", "name_detail", "name_from", "name_source",
              "reference", "thumbnail")
NO_LENGTH = {"value_m": None, "source": None, "detail": None}
SUM_BASIS = ("Computed from the two single-input paths, not measured: each "
             "point adds the two single-input tip displacements (and the two "
             "tip rotation vectors) at those inputs.")
SERIES_BASIS = ("Computed from the two single-input paths, not measured: the "
                "far segment's motion ({far} alone) is carried on the near "
                "segment's tip ({near} alone), turned by the near segment's "
                "tip rotation before the two displacements are added, and "
                "the two tip rotations are composed.")
NUMBER_WORDS = {2: "two", 3: "three", 4: "four", 5: "five", 6: "six",
                7: "seven", 8: "eight", 9: "nine", 10: "ten", 11: "eleven",
                12: "twelve"}
# order of the quantities in a surface grid and in a test point
SURFACE_KEYS = ["axial_m", "side_y_m", "side_z_m", "bending_m", "twist_rad",
                "input_share"] + LENGTH_KEYS
LEG_KEYS = ["label", "channel", "lead_in_from_origin", "input", "values",
            "length_sensitivity"]

# ---- scaled quantities (the document's "scaled" block) ----
FORM_PRESSURE = "pressure over stiffness"
FORM_TENSION = "tension over stiffness times length squared"
FORM_PULL = "pull over length"
SCALED_NOTE = (
    "Quantities with size and material taken out. For one geometry the shape "
    "depends only on the input over the body's stiffness, and at the same "
    "shape the force grows with E times L squared (E: Young's modulus of the "
    "body; L: body length). So tip travel over L, the scaled input, and "
    "force over E L squared let actuators of different size and material be "
    "compared. Axial and sideways tip travel over L are the quantities "
    "axial_share and bending_share in the main list; they are not repeated "
    "here. The scaled input mixes forms: a pressure over E, a cable tension "
    "over E L squared, or a cable pull over L; each leg says which. A "
    "simulated pressure is first corrected as data/map_pressure_scale.json "
    "says, for the records it lists; that correction has not been ruled on. "
    "Simulated magnitudes are comparative, not predictive.")
SCALED_QUANTITIES = [
    {"key": "input_scaled", "label": "Input, scaled", "unit": "",
     "group": INPUT, "estimate": False,
     "definition": (
         "The leg's own input made into a pure number with the body's "
         "stiffness and length, in the form that suits the kind of input: a "
         "pressure over the body's Young's modulus E; a cable tension over E "
         "times the body length squared; a cable pull (a length) over the "
         "body length, which needs no stiffness. The axis mixes these forms, "
         "and each leg names its own (input_scaled_form) and spells out its "
         "rule (input_scaled_definition). A simulated pressure is first "
         "multiplied by the correction data/map_pressure_scale.json lists for "
         "that record (1 / dt for the scene's pressure, which has not been "
         "ruled on); a published pressure is used as stated. On a leg with "
         "another input held, only the leg's own input is scaled. Null for "
         "the whole leg where the form needs a modulus or a length the "
         "actuator does not have, or the input is of a kind with no rule; "
         "each leg says why.")},
    {"key": "side_y_share", "label": "Sideways travel, y ÷ length",
     "unit": "", "group": SHARE_OF_LENGTH, "estimate": False,
     "definition": (
         "Tip displacement from rest along the record's y axis divided by the "
         "body length. Null for the whole leg where the actuator has no "
         "length: not measured, not zero.")},
    {"key": "side_z_share", "label": "Sideways travel, z ÷ length",
     "unit": "", "group": SHARE_OF_LENGTH, "estimate": False,
     "definition": (
         "Tip displacement from rest along the record's z axis divided by the "
         "body length. Null for the whole leg where the actuator has no "
         "length: not measured, not zero.")},
]
SCALED_KEYS = [q["key"] for q in SCALED_QUANTITIES]
SCALED_FORCE_QUANTITIES = [
    {"key": "size_scaled", "label": "Force size ÷ (E·L²)", "unit": "",
     "definition": (
         "The size of the force with the tip held (size_N, which is the "
         "whole force or only one part of it, as size_is says) divided by the "
         "body's Young's modulus times the body length squared.")},
    {"key": "axial_scaled", "label": "Axial force ÷ (E·L²)", "unit": "",
     "definition": (
         "The axial part of the force with the tip held, signed as stored, "
         "divided by the body's Young's modulus times the body length "
         "squared.")},
    {"key": "sideways_scaled", "label": "Sideways force ÷ (E·L²)",
     "unit": "",
     "definition": (
         "The size of the sideways part of the force with the tip held (from "
         "the y and z parts where the whole vector is stored, otherwise the "
         "stored sideways size) divided by the body's Young's modulus times "
         "the body length squared.")},
    {"key": "input_scaled", "label": "Input, scaled", "unit": "",
     "definition": (
         "Each input at the held-tip levels, scaled by the same rule as the "
         "path's scaled input, one value per input channel; null where a "
         "channel cannot be scaled.")},
]
SCALED_FORCE_KEYS = [q["key"] for q in SCALED_FORCE_QUANTITIES]
NO_MODULUS = ("The record states no Young's modulus for the body, and none "
              "is declared in data/map_stiffness.json, so its scaled input "
              "and scaled force are not formed.")
HAND_PLACED_MODULUS = ("Placed by hand: there is no record to read a "
                       "modulus or an input from, so nothing is scaled.")


def _rel(path):
    return os.path.relpath(path, ROOT).replace("\\", "/")


def _finger_facts(rec, quality):
    """(physics, has_force, flags), from Finger as the flat figure does; if
    Finger cannot read the record, from metadata.actuation_physics and
    whether a blocked block holds data."""
    try:
        f = Finger(rec, quality)
        return f.physics, f.b is not None, f.flags()
    except Exception:  # noqa: BLE001 - any record Finger cannot read
        physics = (rec.get("metadata") or {}).get("actuation_physics")
        blocked = rec.get("blocked") or {}
        has_force = any(isinstance(v, list) and len(v) > 0
                        for k, v in blocked.items() if k != "u")
        flags = [f"{k}:{v}" for k, v in sorted((quality or {}).items())]
        return physics, has_force, flags


def _channels(rec):
    return (rec.get("input") or {}).get("channels") or []


def _is_multi_channel(rec):
    """True when free.u holds more than one value per level. (Single-input
    records store free.u as one-value lists, [[u0], [u1], ...].)"""
    u = (rec.get("free") or {}).get("u") or []
    return len(u) > 0 and isinstance(u[0], list) and len(u[0]) > 1


def split_legs(U):
    """For an (L, C) array of input rows: [(channel, {other channel: held
    value}, row indices ordered by the size of the channel's input)], one
    per channel and per distinct setting of all the other channels, kept if
    it has at least MIN_LEG_LEVELS rows and no two rows with the same size
    of the channel's input. Ordering by size, not by signed value, keeps a
    vacuum (negative) input rising away from rest."""
    L, C = U.shape
    out = []
    for c in range(C):
        others = [j for j in range(C) if j != c]
        groups = {}
        for i in range(L):
            groups.setdefault(tuple(float(U[i, j]) for j in others), []).append(i)
        for setting in sorted(groups):
            rows = sorted(groups[setting], key=lambda i: abs(float(U[i, c])))
            sizes = [abs(float(U[i, c])) for i in rows]
            if len(rows) < MIN_LEG_LEVELS or len(set(sizes)) != len(sizes):
                continue
            out.append((c, dict(zip(others, setting)), rows))
    return out


def channel_tops(rows):
    """Per input channel, the largest size of input it takes in `rows` (the
    record's free rows, each a list with one value per channel)."""
    a = np.abs(np.asarray(rows, float).reshape(len(rows), -1))
    return [float(v) for v in a.max(axis=0)]


def input_share(row, tops):
    """The largest, over the channels, of |input| / that channel's largest
    |input| in the record (tops, from channel_tops). A channel that is zero
    in every row adds nothing."""
    return max([abs(float(v)) / t for v, t in zip(row, tops) if t > 0.0],
               default=0.0)


def _positive_number(v):
    return (isinstance(v, (int, float)) and not isinstance(v, bool)
            and np.isfinite(v) and v > 0)


def record_length(rec):
    """The record's own metadata.length_m where it is a positive number,
    otherwise None."""
    v = (rec.get("metadata") or {}).get("length_m")
    return float(v) if _positive_number(v) else None


def body_shape(xyz, length_m):
    """The quantities that need the body length, for one tip displacement
    [x, y, z] from rest, in LENGTH_KEYS order; every value None where
    length_m is None. The rest tip is (L, 0, 0) and the body is taken as one
    even arc from the base to the tip: phi is the angle at the base between
    the x axis and the line to the tip, the bend is 2 phi and the arc's
    length is c phi / sin(phi) for the straight distance c from base to tip
    (c where phi is 0). The four estimates are None where c is below
    CHORD_FLOOR of L, or where the tip lies straight behind the base (phi is
    pi, where no arc reaches it)."""
    out = dict.fromkeys(LENGTH_KEYS)
    if length_m is None:
        return out
    L = float(length_m)
    dx, dy, dz = (float(v) for v in xyz)
    side = float(np.hypot(dy, dz))
    out["axial_share"] = dx / L
    out["bending_share"] = side / L
    c = float(np.hypot(L + dx, side))
    if c < CHORD_FLOOR * L or (side == 0.0 and L + dx < 0.0):
        return out
    phi = float(np.arctan2(side, L + dx))
    bend = 2.0 * phi
    out["bend_rad"] = bend
    # np.sinc(t) is sin(pi t) / (pi t), and 1 at t = 0
    out["stretch_share"] = c / float(np.sinc(phi / np.pi)) / L - 1.0
    out["bend_y_rad"] = bend * dy / side if side > 0.0 else 0.0
    out["bend_z_rad"] = bend * dz / side if side > 0.0 else 0.0
    return out


def _length_values(disp, length_m):
    """{key: list per level, or None for the whole leg without a length}
    for the keys in LENGTH_KEYS."""
    if length_m is None:
        return dict.fromkeys(LENGTH_KEYS)
    shapes = [body_shape(r, length_m) for r in disp]
    return {k: [s[k] for s in shapes] for k in LENGTH_KEYS}


def length_sensitivity(disp, length_m):
    """{key: list per level} for the keys in SENSITIVITY_KEYS: half the
    difference between the value with the length LENGTH_TOLERANCE longer
    and LENGTH_TOLERANCE shorter, as a size. None at a level where the value
    is None at the length or at either changed length; None as a whole
    without a length."""
    if length_m is None:
        return None
    out = {k: [] for k in SENSITIVITY_KEYS}
    for r in disp:
        mid = body_shape(r, length_m)
        lo = body_shape(r, length_m * (1.0 - LENGTH_TOLERANCE))
        hi = body_shape(r, length_m * (1.0 + LENGTH_TOLERANCE))
        for k in SENSITIVITY_KEYS:
            vals = (mid[k], lo[k], hi[k])
            out[k].append(None if any(v is None for v in vals)
                          else abs(hi[k] - lo[k]) / 2.0)
    return out


def tip_tilt(rotvec):
    """The angle between the tip's x axis, turned by the rotation vector,
    and the base's x axis."""
    x = Rotation.from_rotvec(np.asarray(rotvec, float)).apply([1.0, 0.0, 0.0])
    return float(np.arctan2(np.hypot(x[1], x[2]), x[0]))


def _leg(label, channel, ch, u_vals, disp, rotvec, all_others_zero, shares,
         length_m=None):
    """One leg from its inputs (list), tip displacements (list of [x, y, z]),
    rotation vectors (list of [x, y, z], or None), input shares (list, from
    input_share) and body length (or None). The rows are put in order of the
    size of the input (a stable sort, so rows already in that order keep
    it), and the same order is applied to every array. (leg, None) or (None,
    reason). The leg carries the top level's full rotation vector under
    "_top_rotvec" (None without rotation), which legs_of takes off again."""
    sizes = [abs(float(v)) for v in u_vals]
    if len(set(sizes)) != len(sizes):
        return None, f"leg '{label}' repeats an input size, so it has no order"
    order = sorted(range(len(sizes)), key=lambda k: sizes[k])
    u_vals = [u_vals[k] for k in order]
    disp = [disp[k] for k in order]
    rotvec = None if rotvec is None else [rotvec[k] for k in order]
    shares = [shares[k] for k in order]
    top = sizes[order[-1]]
    if top == 0.0:
        return None, f"leg '{label}' has only zero input, so it has no input range"
    first = disp[0]             # the first drawn row, the lowest input share
    return {
        "label": label,
        "channel": channel,
        "lead_in_from_origin": bool(all_others_zero
                                    and any(float(v) != 0.0 for v in first)),
        "input": {"name": ch.get("name"), "unit": ch.get("unit"),
                  "values": [float(v) for v in u_vals]},
        "values": {
            "axial_m": [float(r[0]) for r in disp],
            "bending_m": [float(v) for v in np.hypot(
                [float(r[1]) for r in disp], [float(r[2]) for r in disp])],
            "twist_rad": (None if rotvec is None
                          else [float(r[0]) for r in rotvec]),
            "side_y_m": [float(r[1]) for r in disp],
            "side_z_m": [float(r[2]) for r in disp],
            "input_share": [float(s) for s in shares],
            **_length_values(disp, length_m),
        },
        "length_sensitivity": length_sensitivity(disp, length_m),
        "_top_rotvec": (None if rotvec is None
                        else [float(v) for v in rotvec[-1]]),
    }, None


def _rotvec(free, n):
    r = free.get("tip_rotvec")
    if not r:
        return None
    a = np.asarray(r, float)
    if a.ndim != 2 or a.shape != (n, 3):
        return None
    return r


def legs_of(rec, length_m=None):
    """(legs, None) or (None, reason). length_m is the body length; left
    out, the record's own metadata.length_m is used (see record_length)."""
    legs, _, why = legs_and_top_rotvecs(rec, length_m)
    return (None, why) if legs is None else (legs, None)


def legs_and_top_rotvecs(rec, length_m=None):
    """(legs, the full tip rotation vector at each leg's top level, None) or
    (None, None, reason); a rotation vector is None where the record holds
    no tip rotation. length_m as in legs_of."""
    if length_m is None:
        length_m = record_length(rec)
    legs, why = _legs(rec, length_m)
    if legs is None:
        return None, None, why
    return legs, [leg.pop("_top_rotvec") for leg in legs], None


def _legs(rec, length_m):
    free = rec.get("free") or {}
    chans = _channels(rec)
    if not _is_multi_channel(rec):
        u, y, why = free_curve(rec)
        if why is not None:
            return None, why
        ch = chans[0] if chans else {}
        disp = free["tip_displacement_m"]
        tops = channel_tops([[v] for v in u])
        leg, why = _leg(ch.get("name") or "input", 0, ch, list(u), disp,
                        _rotvec(free, len(disp)), True,
                        [input_share([v], tops) for v in u], length_m)
        return (None, why) if leg is None else ([leg], None)

    U = np.asarray(free["u"], float)
    disp = free.get("tip_displacement_m") or []
    y = np.asarray(disp, float)
    if U.ndim != 2 or U.shape[1] != len(chans):
        return None, (f"free.u rows do not hold one value per input channel "
                      f"(shape {list(U.shape)}, {len(chans)} channels)")
    if y.ndim != 2 or y.shape != (len(U), 3):
        return None, (f"free tip curve is not one 3D tip position per input "
                      f"row ({len(U)} rows, shape {list(y.shape)})")
    rot = _rotvec(free, len(U))
    tops = channel_tops(free["u"])
    legs = []
    for c, held, rows in split_legs(U):
        name = chans[c].get("name") or f"input {c}"
        parts = [f"{chans[j].get('name') or f'input {j}'} "
                 f"{'at' if v == 0.0 else 'held at'} {v:g} "
                 f"{chans[j].get('unit') or ''}".rstrip()
                 for j, v in held.items()]
        zero = all(v == 0.0 for v in held.values())
        label = name + (" alone, with " if zero else ", with ") + ", ".join(parts)
        leg, why = _leg(label, c, chans[c], [free["u"][i][c] for i in rows],
                        [disp[i] for i in rows],
                        None if rot is None else [rot[i] for i in rows], zero,
                        [input_share(free["u"][i], tops) for i in rows],
                        length_m)
        if leg is None:
            return None, why
        legs.append(leg)
    if not legs:
        return None, (f"no input channel has {MIN_LEG_LEVELS} or more rows "
                      f"with the other channels held fixed")
    return legs, None


def bend_check(legs, top_rotvecs, length_m):
    """Per leg, at its top level: the tilt of the tip measured from the
    stored tip rotation (tip_tilt) beside the estimated bend. None where the
    record holds no tip rotation or the actuator has no length."""
    if length_m is None or any(r is None for r in top_rotvecs):
        return None
    return [{"leg": leg["label"], "measured_tilt_rad": tip_tilt(r),
             "estimated_bend_rad": leg["values"]["bend_rad"][-1]}
            for leg, r in zip(legs, top_rotvecs)]


def _quantities(xyz, twist, share, length_m=None):
    """The surface quantities for one tip displacement [x, y, z], one twist
    (or None), one input share and the body length (or None), in
    SURFACE_KEYS order."""
    x, y, z = (float(v) for v in xyz)
    return {"axial_m": x, "side_y_m": y, "side_z_m": z,
            "bending_m": float(np.hypot(y, z)),
            "twist_rad": None if twist is None else float(twist),
            "input_share": float(share),
            **body_shape(xyz, length_m)}


def _alone_at(rid, c, v, rows, U, Y, R):
    """The tip displacement [x, y, z] and tip rotation vector [x, y, z] (or
    None) of channel c acting alone at input v, from its leg `rows` (ordered
    by size): the stored row where v is one of the leg's inputs, otherwise
    the leg interpolated linearly over its inputs, each component on its
    own. (xyz, rotvec, interpolated). Raises ValueError for an input outside
    the leg's range: never extrapolated or clamped."""
    for i in rows:
        if float(U[i, c]) == float(v):
            return ([float(t) for t in Y[i]],
                    None if R is None else [float(t) for t in R[i]], False)
    by_value = sorted(rows, key=lambda i: U[i, c])
    xs = [float(U[i, c]) for i in by_value]
    if not xs[0] <= float(v) <= xs[-1]:
        raise ValueError(f"{rid}: input {v:g} of channel {c} lies outside "
                         f"its single-input leg ({xs[0]:g} to {xs[-1]:g}), "
                         f"so the surface cannot predict it")

    def line(A):
        return [float(np.interp(float(v), xs, [float(A[i, k]) for i in by_value]))
                for k in range(3)]
    return line(Y), None if R is None else line(R), True


def combine(rule, d0, r0, d1, r1, base_to_tip=None):
    """The tip displacement [x, y, z] and rotation vector (or None) with
    channel 0 alone giving (d0, r0) and channel 1 alone giving (d1, r1),
    combined by `rule` (see the module docstring). Under "series",
    base_to_tip names the channel nearer the base first. Where one segment
    has not turned at all (its rotation vector is exactly zero, as at zero
    input), the other's rotation and the plain sum of displacements are
    used as they are, so the surface's edges are the single-input legs to
    the last bit."""
    if rule == "sum":
        return ([a + b for a, b in zip(d0, d1)],
                None if r0 is None else [a + b for a, b in zip(r0, r1)])
    if rule != "series":
        raise ValueError(f"unknown two-input rule {rule!r}")
    if r0 is None or r1 is None:
        raise ValueError("the series rule needs the tip rotation")
    (dA, rA), (dB, rB) = (((d0, r0), (d1, r1)) if list(base_to_tip) == [0, 1]
                          else ((d1, r1), (d0, r0)))
    if not any(rA):            # the near segment has not turned
        return [a + b for a, b in zip(dA, dB)], [float(v) for v in rB]
    RA = Rotation.from_rotvec(rA)
    xyz = [a + float(b) for a, b in zip(dA, RA.apply(dB))]
    if not any(rB):            # the far segment has not turned
        return xyz, [float(v) for v in rA]
    return xyz, [float(v) for v in (RA * Rotation.from_rotvec(rB)).as_rotvec()]


XYZ = ("axial_m", "side_y_m", "side_z_m")


def _gaps(measured, predicted):
    """(position gap, gap over the length of the measured displacement or
    None where it has not moved, |twist gap| or None)."""
    m = np.array([measured[k] for k in XYZ])
    p = np.array([predicted[k] for k in XYZ])
    gap = float(np.linalg.norm(m - p))
    length = float(np.linalg.norm(m))
    twist = (None if measured["twist_rad"] is None
             or predicted["twist_rad"] is None
             else abs(measured["twist_rad"] - predicted["twist_rad"]))
    return gap, (gap / length if length > 0.0 else None), twist


def _worst(tests, share_key, twist_key):
    """(largest position gap share, largest twist gap over the largest
    |measured twist|), each None where there is nothing to take it over."""
    shares = [t[share_key] for t in tests if t[share_key] is not None]
    gaps = [t[twist_key] for t in tests if t[twist_key] is not None]
    top_twist = max([abs(t["measured"]["twist_rad"]) for t in tests
                     if t["measured"]["twist_rad"] is not None], default=0.0)
    return (max(shares) if shares else None,
            max(gaps) / top_twist if gaps and top_twist > 0.0 else None)


def worst_gap_share_by_key(tests):
    """For each estimate in ESTIMATE_KEYS: the largest |measured -
    predicted| over the test points divided by the largest |measured| over
    the same points, skipping points where either value is None; None where
    no point is left or the largest |measured| is 0."""
    out = {}
    for k in ESTIMATE_KEYS:
        pairs = [(t["measured"][k], t["predicted"][k]) for t in tests
                 if t["measured"][k] is not None
                 and t["predicted"][k] is not None]
        top = max([abs(m) for m, _ in pairs], default=0.0)
        out[k] = (max(abs(m - p) for m, p in pairs) / top
                  if pairs and top > 0.0 else None)
    return out


def tests_note(tests, chans):
    """One sentence saying where the test points lie, or None without any."""
    if not tests:
        return None

    def name(c):
        ch = chans[c]
        return ch.get("name") or f"input {c}"

    def amount(c, v):
        return f"{v:g} {chans[c].get('unit') or ''}".rstrip()

    if len(tests) == 1:
        p0, p1 = tests[0]["inputs"]
        return (f"The one test point has the {name(0)} at {amount(0, p0)} and "
                f"the {name(1)} at {amount(1, p1)}.")
    n = NUMBER_WORDS.get(len(tests), str(len(tests)))
    for c in (0, 1):
        vals = {t["inputs"][c] for t in tests}
        if len(vals) == 1:
            return (f"All {n} have the {name(c)} held at "
                    f"{amount(c, vals.pop())}.")
    return f"The {n} test points are spread over both inputs."


def load_extra_checks(path, known_ids):
    """{record id: {"file", "why", "rows", "left_out"}} from
    data/map_extra_check_points.json, checked. "rows" holds one (input 0,
    input 1, [x, y, z] tip displacement in metres on the record's axes) per
    row the file marks as simulated, the rest row included; "left_out"
    counts the other rows by their mark. None for `path` means no such
    file. Raises ValueError for an id that no record or draft record has, a
    missing or extra key, a file whose sha256 is not the pinned one, signs
    that are not 1 or -1, a missing column, two rows at the same inputs, or
    no simulated row with both inputs at zero."""
    if path is None:
        return {}
    with open(path, encoding="utf-8") as fo:
        doc = json.load(fo)
    out = {}
    for rid, e in (doc.get("records") or {}).items():
        who = f"extra check points for {rid}"
        if rid not in known_ids:
            raise ValueError(f"{who}: no record or draft record has that id")
        if not isinstance(e, dict) or sorted(e) != sorted(EXTRA_KEYS):
            raise ValueError(f"{who}: the entry must hold exactly the keys "
                             f"{list(EXTRA_KEYS)}")
        if not isinstance(e["why"], str) or not e["why"].strip():
            raise ValueError(f"{who} has no 'why' saying where the file "
                             f"comes from")
        signs = e["to_record_axes"]
        if not (isinstance(signs, list) and len(signs) == 3
                and all(type(v) is int and v in (1, -1) for v in signs)):
            raise ValueError(f"{who}: to_record_axes must be three signs, "
                             f"each 1 or -1, not {signs!r}")
        use = e["use"]
        if not (isinstance(use, dict) and sorted(use) == ["column", "value"]):
            raise ValueError(f"{who}: 'use' must hold a column and a value")
        if len(e["inputs"]) != 2 or len(e["position_mm"]) != 3:
            raise ValueError(f"{who}: needs two input columns and three "
                             f"position columns")
        full = os.path.join(ROOT, *e["file"].split("/"))
        got = sha256(full)
        if got != e["sha256"]:
            raise ValueError(f"{who}: {e['file']} has sha256 {got}, not the "
                             f"pinned {e['sha256']}; the file has changed, so "
                             f"look at the entry again")
        with open(full, encoding="utf-8", newline="") as fo:
            table = list(csv.DictReader(fo))
        cols = list(e["inputs"]) + list(e["position_mm"]) + [use["column"]]
        missing = [c for c in cols if not table or c not in table[0]]
        if missing:
            raise ValueError(f"{who}: {e['file']} has no column {missing}")
        rows, left_out, seen = [], {}, set()
        for t in table:
            mark = t[use["column"]]
            if mark != use["value"]:
                left_out[mark] = left_out.get(mark, 0) + 1
                continue
            p = tuple(float(t[c]) for c in e["inputs"])
            if p in seen:
                raise ValueError(f"{who}: {e['file']} has two rows at inputs "
                                 f"{list(p)}")
            seen.add(p)
            rows.append((p, [float(t[c]) for c in e["position_mm"]]))
        rest = [xyz for p, xyz in rows if p == (0.0, 0.0)]
        if not rest:
            raise ValueError(f"{who}: {e['file']} has no simulated row with "
                             f"both inputs at zero, so there is no rest "
                             f"position to measure from")
        out[rid] = {"file": e["file"], "why": e["why"], "left_out": left_out,
                    "rows": [(p[0], p[1],
                              [sg * (v - r0) * 1e-3 for sg, v, r0
                               in zip(signs, xyz, rest[0])])
                             for p, xyz in rows]}
    return out


def extra_checks_note(n, file, left_out):
    """One or two sentences saying what the extra check points are."""
    note = (f"{n} further simulated point{'' if n == 1 else 's'} with both "
            f"inputs on, read from {file}: tip position only, no tip "
            f"rotation.")
    for mark in sorted(left_out):
        k = left_out[mark]
        note += (f" {k} row{'' if k == 1 else 's'} of that file marked "
                 f"'{mark}' {'is' if k == 1 else 'are'} left out: not "
                 f"simulated.")
    return note


def surface_of(rec, decl, length_m=None, extra=None):
    """The computed surface of tip positions for a two-input record under its
    declared rule (decl, an entry of data/map_two_input_rules.json), or
    None where nothing is declared, or where the record lacks a leg for
    each channel with the other at exactly 0 that starts at zero input.
    Raises ValueError for a declaration on a record without two input
    channels, or a series declaration on a record without tip rotation. See
    the module docstring for what it holds. length_m is the body length;
    left out, the record's own metadata.length_m is used (see
    record_length). extra is the record's entry from load_extra_checks, or
    None: see the module docstring."""
    if decl is None:
        return None
    if length_m is None:
        length_m = record_length(rec)
    rid = rec["id"]
    rule, order = decl["rule"], decl.get("base_to_tip")
    free = rec.get("free") or {}
    chans = _channels(rec)
    if len(chans) != 2 or not _is_multi_channel(rec):
        raise ValueError(f"{rid}: a two-input rule is declared, but the "
                         f"record does not have two input channels")
    U = np.asarray(free["u"], float)
    Y = np.asarray(free.get("tip_displacement_m") or [], float)
    if U.ndim != 2 or U.shape[1] != 2 or Y.shape != (len(U), 3):
        return None
    rot = _rotvec(free, len(U))
    R = None if rot is None else np.asarray(rot, float)
    if rule == "series" and R is None:
        raise ValueError(f"{rid}: declared 'series', but the record holds no "
                         f"tip rotation (free.tip_rotvec), which the series "
                         f"rule needs to carry one segment on the other")
    alone = {}
    for c, held, rows in split_legs(U):
        if all(v == 0.0 for v in held.values()) and U[rows[0], c] == 0.0:
            alone[c] = rows
    if sorted(alone) != [0, 1]:
        return None
    tops = channel_tops(free["u"])
    a0, a1 = alone[0], alone[1]

    def row(i):
        return [float(t) for t in Y[i]], None if R is None else [float(t) for t in R[i]]

    def quantities(xyz, rv, share):
        return _quantities(xyz, None if rv is None else rv[0], share,
                           length_m)

    grid = {k: [] for k in SURFACE_KEYS}
    for i in a0:
        line = {k: [] for k in SURFACE_KEYS}
        for j in a1:
            xyz, rv = combine(rule, *row(i), *row(j), order)
            q = quantities(xyz, rv, input_share([U[i, 0], U[j, 1]], tops))
            for k in SURFACE_KEYS:
                line[k].append(q[k])
        for k in SURFACE_KEYS:
            grid[k].append(line[k])
    if R is None:
        grid["twist_rad"] = None
    if length_m is None:
        for k in LENGTH_KEYS:
            grid[k] = None

    tests = []
    for r in range(len(U)):
        p0, p1 = float(U[r, 0]), float(U[r, 1])
        if p0 == 0.0 or p1 == 0.0:
            continue
        d0, r0, int0 = _alone_at(rid, 0, p0, a0, U, Y, R)
        d1, r1, int1 = _alone_at(rid, 1, p1, a1, U, Y, R)
        share = input_share([p0, p1], tops)
        measured = quantities(*row(r), share)
        predicted = quantities(*combine(rule, d0, r0, d1, r1, order), share)
        gap, gap_share, twist_gap = _gaps(measured, predicted)
        t = {"inputs": [p0, p1],
             "interpolated": bool(int0 or int1),
             "measured": measured,
             "predicted": predicted,
             "position_gap_m": gap,
             # null where the measured tip has not moved (no length to share)
             "position_gap_share": gap_share,
             "twist_gap_rad": twist_gap,
             "predicted_sum": None, "sum_position_gap_m": None,
             "sum_position_gap_share": None, "sum_twist_gap_rad": None}
        if rule != "sum":      # the plain sum, kept for comparison
            t["predicted_sum"] = quantities(*combine("sum", d0, r0, d1, r1), share)
            (t["sum_position_gap_m"], t["sum_position_gap_share"],
             t["sum_twist_gap_rad"]) = _gaps(measured, t["predicted_sum"])
        tests.append(t)

    extra_checks = None
    if extra is not None:
        held = {(float(U[r, 0]), float(U[r, 1])): r for r in range(len(U))}
        shared, points = [0, 0], []
        for p0, p1, xyz in extra["rows"]:
            r = held.get((p0, p1))
            if r is not None:
                off = float(np.linalg.norm(np.asarray(xyz) - Y[r]))
                if off > EXTRA_MATCH_M:
                    raise ValueError(
                        f"{rid}: {extra['file']} at inputs [{p0:g}, {p1:g}] "
                        f"is {off:.3g} m from the record's own tip "
                        f"displacement there, so its axes or its rest "
                        f"position are not the record's")
                for c, v in enumerate((p0, p1)):
                    shared[c] += v != 0.0
                continue
            if p0 == 0.0 or p1 == 0.0:
                continue
            d0, r0, int0 = _alone_at(rid, 0, p0, a0, U, Y, R)
            d1, r1, int1 = _alone_at(rid, 1, p1, a1, U, Y, R)
            share = input_share([p0, p1], tops)
            measured = quantities(xyz, None, share)
            predicted = quantities(*combine(rule, d0, r0, d1, r1, order), share)
            gap, gap_share, _ = _gaps(measured, predicted)
            points.append({"inputs": [p0, p1],
                           "interpolated": bool(int0 or int1),
                           "measured": measured, "predicted": predicted,
                           "position_gap_m": gap,
                           "position_gap_share": gap_share})
        if min(shared) < EXTRA_MIN_SHARED:
            raise ValueError(
                f"{rid}: {extra['file']} shares {shared[0]} and {shared[1]} "
                f"rows with the record with each input on; at least "
                f"{EXTRA_MIN_SHARED} of each are needed to check its axes")
        shares = [t["position_gap_share"] for t in points
                  if t["position_gap_share"] is not None]
        extra_checks = {
            "file": extra["file"], "why": extra["why"],
            "note": extra_checks_note(len(points), extra["file"],
                                      extra["left_out"]),
            "shared_rows_matched": sum(1 for p0, p1, _ in extra["rows"]
                                       if (p0, p1) in held),
            "left_out": dict(extra["left_out"]),
            "points": points,
            "worst_position_gap_m": max([t["position_gap_m"] for t in points],
                                        default=None),
            "worst_position_gap_share": max(shares) if shares else None,
        }

    worst = _worst(tests, "position_gap_share", "twist_gap_rad")
    worst_sum = ((None, None) if rule == "sum" else
                 _worst(tests, "sum_position_gap_share", "sum_twist_gap_rad"))
    if rule == "series":
        near, far = (chans[c].get("name") or f"input {c}" for c in order)
        basis = SERIES_BASIS.format(near=near, far=far)
    else:
        basis = SUM_BASIS
    return {
        "rule": rule,
        "basis": basis,
        "inputs": [{"name": chans[c].get("name"), "unit": chans[c].get("unit"),
                    "values": [float(U[i, c]) for i in alone[c]]}
                   for c in (0, 1)],
        "values": grid,
        "tests": tests,
        "tests_note": tests_note(tests, chans),
        "worst_position_gap_share": worst[0],
        "worst_twist_gap_share": worst[1],
        "worst_sum_position_gap_share": worst_sum[0],
        "worst_sum_twist_gap_share": worst_sum[1],
        "worst_gap_share_by_key": worst_gap_share_by_key(tests),
        "extra_checks": extra_checks,
    }


def load_two_input_rules(path, known_ids):
    """{record id: declaration} from data/map_two_input_rules.json, checked.
    Raises ValueError for an id that no record or draft record has, an
    unknown rule, a series entry without both channel indices (0 and 1,
    once each) in base_to_tip, a sum entry with an order, an entry without
    a 'why', or any other key."""
    with open(path, encoding="utf-8") as fo:
        doc = json.load(fo)
    out = {}
    for rid, e in (doc.get("records") or {}).items():
        who = f"two-input rule for {rid}"
        if rid not in known_ids:
            raise ValueError(f"{who}: no record or draft record has that id")
        if not isinstance(e, dict):
            raise ValueError(f"{who} is not an object")
        extra = sorted(set(e) - {"rule", "base_to_tip", "why"})
        if extra:
            raise ValueError(f"{who} has keys it should not: {extra}")
        if e.get("rule") not in RULES:
            raise ValueError(f"{who}: rule must be one of {list(RULES)}, "
                             f"not {e.get('rule')!r}")
        why = e.get("why")
        if not isinstance(why, str) or not why.strip():
            raise ValueError(f"{who} has no 'why' saying how the arrangement "
                             f"is known")
        b = e.get("base_to_tip")
        if e["rule"] == "series":
            if not (isinstance(b, list) and all(type(v) is int for v in b)
                    and sorted(b) == [0, 1]):
                raise ValueError(f"{who}: base_to_tip must list the channel "
                                 f"indices 0 and 1 once each, nearer the base "
                                 f"first, not {b!r}")
        elif "base_to_tip" in e:
            raise ValueError(f"{who}: a 'sum' rule has no base-to-tip order")
        out[rid] = e
    return out


def load_lengths(path, record_lengths):
    """{record id: {"length_m", "source"}} from data/map_lengths.json,
    checked. record_lengths maps every record and draft record id to its own
    length (record_length), so an estimate is refused for a record that
    already holds one. Raises ValueError for an id that no record or draft
    record has, an entry that is not an object, a length_m that is not a
    positive finite number, a source that is missing or empty, or an entry
    for a record whose own metadata.length_m is set."""
    with open(path, encoding="utf-8") as fo:
        doc = json.load(fo)
    out = {}
    for rid, e in (doc.get("records") or {}).items():
        who = f"length estimate for {rid}"
        if rid not in record_lengths:
            raise ValueError(f"{who}: no record or draft record has that id")
        if not isinstance(e, dict):
            raise ValueError(f"{who} is not an object")
        if record_lengths[rid] is not None:
            raise ValueError(f"{who}: the record already holds its own length "
                             f"({record_lengths[rid]:g} m in "
                             f"metadata.length_m), and an estimate never "
                             f"replaces it")
        v = e.get("length_m")
        if not _positive_number(v):
            raise ValueError(f"{who}: length_m must be a positive number of "
                             f"metres, not {v!r}")
        src = e.get("source")
        if not isinstance(src, str) or not src.strip():
            raise ValueError(f"{who} has no 'source' saying where the number "
                             f"comes from")
        out[rid] = {"length_m": float(v), "source": src}
    return out


def length_of(rec, estimates):
    """The actuator's "length" entry: the record's own metadata.length_m,
    otherwise its estimate from load_lengths, otherwise all None."""
    own = record_length(rec)
    if own is not None:
        return {"value_m": own, "source": "record", "detail": None}
    e = estimates.get(rec["id"])
    if e is not None:
        return {"value_m": e["length_m"], "source": "estimate",
                "detail": e["source"]}
    return dict(NO_LENGTH)


def _materials(rec):
    m = (rec.get("metadata") or {}).get("materials")
    return m if isinstance(m, dict) else {}


def record_modulus(rec):
    """The record's own metadata.materials.young_modulus_body where it is a
    positive number, otherwise None."""
    v = _materials(rec).get("young_modulus_body")
    return float(v) if _positive_number(v) else None


def _check_pin(who, src, keys=PIN_KEYS):
    """Raise ValueError unless src is a pin naming every key in `keys`, with
    a 64-hex sha256 and a positive whole line number."""
    if not isinstance(src, dict):
        raise ValueError(f"{who}: the source must be an object naming "
                         f"{list(keys)}")
    missing = [k for k in keys if k not in src]
    if missing:
        raise ValueError(f"{who}: the source is missing {missing}")
    for k in keys:
        if k == "line":
            continue
        if not isinstance(src[k], str) or not src[k].strip():
            raise ValueError(f"{who}: the source's {k} must be a non-empty "
                             f"string")
    sha = src["sha256"]
    if len(sha) != 64 or any(ch not in "0123456789abcdef" for ch in sha):
        raise ValueError(f"{who}: the source's sha256 is not 64 lower-case "
                         f"hex digits")
    if "line" in keys:
        line = src["line"]
        if not isinstance(line, int) or isinstance(line, bool) or line < 1:
            raise ValueError(f"{who}: the source's line must be a positive "
                             f"whole number, not {line!r}")


def load_stiffness(path, record_moduli):
    """{record id: {"modulus_Pa", "source", "why"}} from
    data/map_stiffness.json, checked. record_moduli maps every record and
    draft record id to its own body modulus (record_modulus), so a declared
    value is refused for a record that already holds one. Raises ValueError
    for an id that no record or draft record has, an entry that is not an
    object, an entry for a record whose own young_modulus_body is set, a
    modulus_Pa that is not a positive finite number, a source that is not a
    full pin (PIN_KEYS), or a missing or empty "why"."""
    with open(path, encoding="utf-8") as fo:
        doc = json.load(fo)
    out = {}
    for rid, e in (doc.get("records") or {}).items():
        who = f"body modulus for {rid}"
        if rid not in record_moduli:
            raise ValueError(f"{who}: no record or draft record has that id")
        if not isinstance(e, dict):
            raise ValueError(f"{who} is not an object")
        if record_moduli[rid] is not None:
            raise ValueError(f"{who}: the record already holds its own body "
                             f"modulus ({record_moduli[rid]:g} Pa in "
                             f"metadata.materials.young_modulus_body), and a "
                             f"declared value never replaces it")
        v = e.get("modulus_Pa")
        if not _positive_number(v):
            raise ValueError(f"{who}: modulus_Pa must be a positive number of "
                             f"pascals, not {v!r}")
        _check_pin(who, e.get("source"))
        why = e.get("why")
        if not isinstance(why, str) or not why.strip():
            raise ValueError(f"{who} has no 'why' saying why that value is "
                             f"the one the run used")
        out[rid] = {"modulus_Pa": float(v), "source": dict(e["source"]),
                    "why": why}
    return out


def _has_pa_channel(rec):
    return any(c.get("unit") == "Pa" for c in _channels(rec))


def load_pressure_scale(path, records):
    """(by record, entries) from data/map_pressure_scale.json, checked.
    records maps every record and draft record id to its record. by record
    is {record id: {"entry", "factor", "dt_s", "ruled"}}, the factor being
    1 / dt_s; entries is {entry name: the entry as written, with "factor"
    added}. Raises ValueError for an entry that is not an object, a dt_s
    that is not a positive number, dt_sources that are not a non-empty list
    of full pins (PIN_KEYS), evidence without a path and sha256, a missing
    or empty "why" or "chosen_by", a "ruled" that is not true or false, an
    empty applies_to, an id that no record or draft record has, an id whose
    record is not a simulation run in the scene SCENE_NAME with an input in
    Pa, and an id listed in two entries."""
    with open(path, encoding="utf-8") as fo:
        doc = json.load(fo)
    by_record, entries = {}, {}
    for name, e in (doc.get("factors") or {}).items():
        who = f"pressure correction {name}"
        if not isinstance(e, dict):
            raise ValueError(f"{who} is not an object")
        dt = e.get("dt_s")
        if not _positive_number(dt):
            raise ValueError(f"{who}: dt_s must be a positive number of "
                             f"seconds, not {dt!r}")
        pins = e.get("dt_sources")
        if not isinstance(pins, list) or not pins:
            raise ValueError(f"{who} has no dt_sources pinning the time step")
        for p in pins:
            _check_pin(who, p)
        ev = e.get("evidence")
        if not isinstance(ev, dict) or not ev.get("path"):
            raise ValueError(f"{who} has no evidence path")
        _check_pin(who, ev, ("path", "sha256"))
        for k in ("why", "chosen_by"):
            if not isinstance(e.get(k), str) or not e[k].strip():
                raise ValueError(f"{who} has no '{k}'")
        if not isinstance(e.get("ruled"), bool):
            raise ValueError(f"{who}: 'ruled' must be true or false")
        ids = e.get("applies_to")
        if not isinstance(ids, list) or not ids:
            raise ValueError(f"{who}: applies_to must list at least one "
                             f"record id")
        factor = 1.0 / float(dt)
        for rid in ids:
            if rid not in records:
                raise ValueError(f"{who}: no record or draft record has the "
                                 f"id {rid}")
            rec = records[rid]
            if ((rec.get("source") or {}).get("kind") != "simulation"
                    or (rec.get("provenance") or {}).get("scene") != SCENE_NAME
                    or not _has_pa_channel(rec)):
                raise ValueError(f"{who}: {rid} is not a simulation run in "
                                 f"{SCENE_NAME} with an input in Pa, so the "
                                 f"scene's pressure correction cannot apply "
                                 f"to it")
            if rid in by_record:
                raise ValueError(f"{who}: {rid} is already corrected by "
                                 f"{by_record[rid]['entry']}")
            by_record[rid] = {"entry": name, "factor": factor,
                              "dt_s": float(dt), "ruled": e["ruled"]}
        entries[name] = {**e, "factor": factor}
    return by_record, entries


def load_force_withheld(path, record_paths):
    """{record id: {"reason", "evidence"}} from data/map_force_withheld.json,
    checked: the records whose force with the tip held is left off the map.
    record_paths maps every record and draft record id to the record file
    the export reads. None for `path` means none is left off. Raises
    ValueError for an id that no record or draft record has, an entry that
    is not an object, a missing or empty reason, evidence that does not pin
    the record file by path and sha256 (or pins another file, or a sha256
    the file no longer has), and a missing or empty probe statement."""
    if path is None:
        return {}
    with open(path, encoding="utf-8") as fo:
        doc = json.load(fo)
    out = {}
    for rid, e in (doc.get("records") or {}).items():
        who = f"withheld force for {rid}"
        if rid not in record_paths:
            raise ValueError(f"{who}: no record or draft record has that id")
        if not isinstance(e, dict):
            raise ValueError(f"{who} is not an object")
        reason = e.get("reason")
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError(f"{who} has no 'reason' saying why the force is "
                             f"left off")
        ev = e.get("evidence")
        _check_pin(who, ev, ("path", "sha256"))
        want = _rel(record_paths[rid])
        if ev["path"] != want:
            raise ValueError(f"{who}: the evidence pins {ev['path']}, but the "
                             f"export reads the record from {want}")
        got = sha256(record_paths[rid])
        if got != ev["sha256"]:
            raise ValueError(f"{who}: {want} has sha256 {got}, not the "
                             f"pinned {ev['sha256']}; the record has changed, "
                             f"so look at the entry again")
        probe = ev.get("probe")
        if not isinstance(probe, str) or not probe.strip():
            raise ValueError(f"{who}: the evidence has no 'probe' statement")
        out[rid] = {"reason": reason, "evidence": dict(ev)}
    return out


def _text(who, e, key):
    """e[key], or ValueError unless it is a non-empty string."""
    v = e.get(key)
    if not isinstance(v, str) or not v.strip():
        raise ValueError(f"{who} has no '{key}'")
    return v


def load_names(path, kinds):
    """{record id: {"display_name", "name_detail", "name_from",
    "name_source", "reference"}} from data/map_names.json, checked: the
    name each actuator is shown by and the reference its card links to.
    kinds maps every record and draft record id to its source kind. A
    reference comes out whole (citation, authors_short, year, doi, url,
    open_copy), or None for a record with none. None for `path` means no
    names. Raises ValueError for an id that no record or draft record has,
    a missing or empty name, detail or name source, a name longer than
    NAME_MAX characters, two records with the same name, a name_from not in
    NAME_FROM, a reference key the file does not hold, a literature record
    with no reference, and a reference without its citation, short
    authors, year, DOI, a link that is https://doi.org/ plus that DOI, or
    the date and way it was checked (or with an open copy that is not an
    https link with a description)."""
    if path is None:
        return {}
    with open(path, encoding="utf-8") as fo:
        doc = json.load(fo)
    refs = {}
    for key, r in (doc.get("references") or {}).items():
        who = f"reference {key}"
        if not isinstance(r, dict):
            raise ValueError(f"{who} is not an object")
        for k in ("citation", "authors_short", "doi", "checked",
                  "checked_how"):
            _text(who, r, k)
        year = r.get("year")
        if not isinstance(year, int) or isinstance(year, bool):
            raise ValueError(f"{who}: the year must be a whole number")
        if r.get("url") != DOI_LINK + r["doi"]:
            raise ValueError(f"{who}: the link must be {DOI_LINK} followed "
                             f"by the DOI")
        oc = r.get("open_copy")
        if oc is not None:
            if (not isinstance(oc, dict)
                    or not str(oc.get("url") or "").startswith("https://")):
                raise ValueError(f"{who}: the open copy must hold an https "
                                 f"link")
            _text(f"{who}'s open copy", oc, "what")
            oc = {"url": oc["url"], "what": oc["what"]}
        refs[key] = {"citation": r["citation"],
                     "authors_short": r["authors_short"], "year": year,
                     "doi": r["doi"], "url": r["url"], "open_copy": oc}
    out, seen = {}, {}
    for rid, e in (doc.get("records") or {}).items():
        who = f"name for {rid}"
        if rid not in kinds:
            raise ValueError(f"{who}: no record or draft record has that id")
        if not isinstance(e, dict):
            raise ValueError(f"{who} is not an object")
        name = _text(who, e, "name")
        if len(name) > NAME_MAX:
            raise ValueError(f"{who}: longer than {NAME_MAX} characters")
        if name in seen:
            raise ValueError(f"{who}: {seen[name]} has the same name, "
                             f"{name!r}")
        seen[name] = rid
        if e.get("name_from") not in NAME_FROM:
            raise ValueError(f"{who}: name_from must be one of "
                             f"{list(NAME_FROM)}")
        key = e.get("reference")
        if key is not None and key not in refs:
            raise ValueError(f"{who}: the file holds no reference {key!r}")
        if key is None and kinds[rid] == "literature":
            raise ValueError(f"{who}: the record is from the literature, so "
                             f"it needs a reference")
        out[rid] = {"display_name": name,
                    "name_detail": _text(who, e, "detail"),
                    "name_from": e["name_from"],
                    "name_source": _text(who, e, "name_source"),
                    "reference": None if key is None else dict(refs[key])}
    return out


def load_thumbnails(path, known, site_dir=SITE_DIR):
    """{record id: {"image", "kind", "basis"}} from data/map_thumbnails.json,
    checked: the small picture of each actuator. known is every record and
    draft record id; site_dir is the folder the image paths are under. For
    kind "mesh" the basis is MESH_BASIS; for "schematic" it is the entry's
    own. None for `path` means no pictures. Raises ValueError for an id
    that no record or draft record has, a kind not in THUMBNAIL_KINDS, an
    image that is not a .png path under site_dir or is not there, a mesh
    picture without pinned sources (repo, commit, path, sha256, and which
    part each draws, exactly one of them the body), and a schematic without
    the name of its drawing and its basis. The pins are checked against the
    mesh repository by the tests, not here: the export needs neither that
    repository nor the drawing libraries."""
    if path is None:
        return {}
    with open(path, encoding="utf-8") as fo:
        doc = json.load(fo)
    out = {}
    for rid, e in (doc.get("records") or {}).items():
        who = f"picture for {rid}"
        if rid not in known:
            raise ValueError(f"{who}: no record or draft record has that id")
        if not isinstance(e, dict):
            raise ValueError(f"{who} is not an object")
        kind = e.get("kind")
        if kind not in THUMBNAIL_KINDS:
            raise ValueError(f"{who}: kind must be one of "
                             f"{list(THUMBNAIL_KINDS)}")
        image = _text(who, e, "image")
        parts = image.split("/")
        if (not image.endswith(".png") or "\\" in image or ":" in image
                or any(q in ("", ".", "..") for q in parts)):
            raise ValueError(f"{who}: the image must be a .png path under "
                             f"site/, written with forward slashes")
        if not os.path.isfile(os.path.join(site_dir, *parts)):
            raise ValueError(f"{who}: site/{image} is not there; draw it "
                             f"with tools/render_map_thumbnails.py")
        if kind == "mesh":
            srcs = e.get("sources")
            if not isinstance(srcs, list) or not srcs:
                raise ValueError(f"{who}: a mesh picture must list its "
                                 f"sources")
            for src in srcs:
                _check_pin(who, src, THUMBNAIL_PIN_KEYS)
                if src.get("draws") not in THUMBNAIL_PARTS:
                    raise ValueError(f"{who}: each source's 'draws' must be "
                                     f"one of {list(THUMBNAIL_PARTS)}")
            if [x["draws"] for x in srcs].count("body") != 1:
                raise ValueError(f"{who}: exactly one source must draw the "
                                 f"body")
            basis = MESH_BASIS
        else:
            _text(who, e, "schematic")
            basis = _text(who, e, "basis")
        out[rid] = {"image": image, "kind": kind, "basis": basis}
    return out


# an actuator with no entry in data/map_names.json is shown by its id
NO_NAME = {"display_name": None, "name_detail": None, "name_from": None,
           "name_source": None, "reference": None}


def stiffness_of(rec, declared):
    """The actuator's body modulus: the record's own
    metadata.materials.young_modulus_body ("source": "record"), otherwise
    its entry from load_stiffness ("source": "declared", with the pin),
    otherwise None with the reason. Where the record also gives
    young_modulus_limiter, it is given with the limiter-to-body ratio."""
    out = {"modulus_Pa": None, "source": None, "detail": None, "pin": None,
           "limiter_modulus_Pa": None, "limiter_over_body": None,
           "reason": None}
    own = record_modulus(rec)
    if own is not None:
        out.update(modulus_Pa=own, source="record",
                   detail="metadata.materials.young_modulus_body in the record")
        lim = _materials(rec).get("young_modulus_limiter")
        if _positive_number(lim):
            out.update(limiter_modulus_Pa=float(lim),
                       limiter_over_body=float(lim) / own)
        return out
    e = declared.get(rec["id"])
    if e is not None:
        out.update(modulus_Pa=e["modulus_Pa"], source="declared",
                   detail=e["why"], pin=e["source"])
        return out
    out["reason"] = NO_MODULUS
    return out


def scaled_input(physics, unit, name, values, E, L, factor):
    """(form, scaled values, definition, reason) for one input channel.
    A pressure (unit Pa) is multiplied by `factor` (the record's pressure
    correction, or None for none) and divided by E. A cable input in metres
    is a pull, divided by L, with no stiffness and no correction. A cable
    input in newtons is a tension, divided by E L squared. Anything else, or
    a form whose E or L is missing, gives (form or None, None, None,
    reason)."""
    what = f"'{name}'" if name else "the input"
    if unit == "Pa":
        if E is None:
            return FORM_PRESSURE, None, None, NO_MODULUS
        f = 1.0 if factor is None else factor
        return FORM_PRESSURE, [float(v) * f / E for v in values], None, None
    if physics == "cable" and unit == "m":
        if L is None:
            return FORM_PULL, None, None, (
                "A cable pull is scaled by the body length, and the actuator "
                "has no length.")
        return FORM_PULL, [float(v) / L for v in values], None, None
    if physics == "cable" and unit == "N":
        if E is None:
            return FORM_TENSION, None, None, NO_MODULUS
        if L is None:
            return FORM_TENSION, None, None, (
                "A cable tension is scaled by the body length, and the "
                "actuator has no length.")
        return FORM_TENSION, [float(v) / (E * L * L) for v in values], None, None
    if unit is None:
        return None, None, None, (
            f"The input {what} has no unit in the record, so no rule for "
            f"scaling it applies.")
    return None, None, None, (
        f"The input {what} is in {unit} on a {physics or 'unnamed'} "
        f"actuator; there is no rule for scaling that kind of input.")


def _modulus_words(st):
    m = f"{st['modulus_Pa']:g} Pa"
    if st["source"] == "record":
        return f"E = {m}, the record's own body modulus"
    p = st["pin"] or {}
    return (f"E = {m}, declared in data/map_stiffness.json from "
            f"{p.get('symbol')} in {p.get('path')} at commit "
            f"{str(p.get('commit'))[:7]}")


def _input_definition(form, unit, name, st, L, corr, held):
    """The plain-words rule for one leg's scaled input."""
    what = f"'{name}' ({unit})"
    if form == FORM_PRESSURE:
        if corr is not None:
            s = (f"Pressure over stiffness: {what} times {corr['factor']:g} "
                 f"(1 / dt, dt {corr['dt_s']:g} s, the correction "
                 f"{corr['entry']} in data/map_pressure_scale.json"
                 + ("" if corr["ruled"] else ", not yet ruled on")
                 + f"), over {_modulus_words(st)}.")
        else:
            s = (f"Pressure over stiffness: {what} as stated, with no "
                 f"correction, over {_modulus_words(st)}.")
    elif form == FORM_PULL:
        s = (f"Pull over length: {what} over L = {L:g} m. No stiffness and "
             f"no correction are used.")
    else:
        s = (f"Tension over stiffness times length squared: {what} over "
             f"{_modulus_words(st)}, times L squared, L = {L:g} m.")
    if held:
        s += (" On this leg another input is held above zero ("
              + ", ".join(held) + "); only this leg's own input is scaled, "
              "so the scaled input starts at 0 here.")
    return s


def scaled_of(a, rec, declared, corrections):
    """The actuator's entry in the document's "scaled" block: its body
    modulus (stiffness_of) and, per leg, the values in SCALED_KEYS with
    each leg's input form and rule, and its held-tip force over E L
    squared. A value that cannot be formed is null and its reason given."""
    if rec is None:
        return {"stiffness": {**stiffness_of({"id": a["id"]}, {}),
                              "reason": HAND_PLACED_MODULUS},
                "legs": [{"label": leg.get("label"),
                          "values": dict.fromkeys(SCALED_KEYS),
                          "input_scaled_form": None,
                          "input_scaled_definition": None,
                          "reasons": dict.fromkeys(SCALED_KEYS,
                                                   HAND_PLACED_MODULUS)}
                         for leg in a["legs"]],
                "force": None}
    st = stiffness_of(rec, declared)
    E, L = st["modulus_Pa"], a["length"]["value_m"]
    corr = corrections.get(rec["id"])
    chans = _channels(rec)
    no_length = ("The actuator has no body length, so no travel over length "
                 "is formed: not measured, not zero.")
    # a multi-input record's legs are split_legs' groups, one for one and in
    # the same order (see _legs), so the held inputs are read from there
    groups = ([h for _, h, _ in split_legs(np.asarray(rec["free"]["u"], float))]
              if _is_multi_channel(rec) else [{} for _ in a["legs"]])
    if len(groups) != len(a["legs"]):
        raise ValueError(f"{rec['id']}: the legs do not match the record's "
                         f"input groups")
    legs = []
    for leg, h in zip(a["legs"], groups):
        u = leg["input"]
        form, vals, _, why = scaled_input(
            a["physics"], u["unit"], u["name"], u["values"], E, L,
            corr["factor"] if corr and u["unit"] == "Pa" else None)
        held = [f"{chans[j].get('name') or f'input {j}'} at "
                f"{v:g} {chans[j].get('unit') or ''}".rstrip()
                for j, v in h.items() if v != 0.0]
        values, reasons = {}, {}
        values["input_scaled"] = vals
        if vals is None:
            reasons["input_scaled"] = why
        for k, src in (("side_y_share", "side_y_m"),
                       ("side_z_share", "side_z_m")):
            if L is None:
                values[k] = None
                reasons[k] = no_length
            else:
                values[k] = [float(v) / L for v in leg["values"][src]]
        legs.append({
            "label": leg["label"], "values": values,
            "input_scaled_form": form if vals is not None else None,
            "input_scaled_definition": (
                None if vals is None else
                _input_definition(form, u["unit"], u["name"], st, L, corr,
                                  held)),
            "reasons": reasons})
    return {"stiffness": st, "legs": legs,
            "force": _scaled_force(a, rec, st, L, corr)}


def _scaled_force(a, rec, st, L, corr):
    """The held-tip force over E L squared, per level, beside the scaled
    inputs at those levels. None where the actuator holds no force."""
    f = a.get("force")
    if f is None:
        return None
    E = st["modulus_Pa"]
    rows = f["input"]["values"]
    n_ch = max((len(r) for r in rows), default=0)
    inputs, reasons = [], {}
    for c in range(n_ch):
        unit = f["input"]["units"][c] if c < len(f["input"]["units"]) else None
        name = f["input"]["names"][c] if c < len(f["input"]["names"]) else None
        _, vals, _, why = scaled_input(
            a["physics"], unit, name, [r[c] for r in rows], E, L,
            corr["factor"] if corr and unit == "Pa" else None)
        inputs.append(vals)
        if vals is None:
            reasons.setdefault("input_scaled", why)
    input_rows = [[None if col is None else col[i] for col in inputs]
                  for i in range(len(rows))]
    out = {"input_scaled": input_rows}
    if E is None or L is None:
        why = st["reason"] if E is None else (
            "The actuator has no body length, so no force over E L squared "
            "is formed.")
        out.update(size_scaled=None, axial_scaled=None, sideways_scaled=None)
        for k in ("size_scaled", "axial_scaled", "sideways_scaled"):
            reasons[k] = why
        out["reasons"] = reasons
        return out
    d = E * L * L
    out["size_scaled"] = [float(v) / d for v in f["size_N"]]
    out["axial_scaled"] = (None if f["axial_N"] is None
                           else [float(v) / d for v in f["axial_N"]])
    if f["vector_N"] is not None:
        side = [float(np.hypot(v[1], v[2])) for v in f["vector_N"]]
    elif f["sideways_size_N"] is not None:
        side = [abs(float(v)) for v in f["sideways_size_N"]]
    else:
        side = None
    out["sideways_scaled"] = None if side is None else [v / d for v in side]
    for k in ("axial_scaled", "sideways_scaled"):
        if out[k] is None:
            reasons[k] = "The record holds no such part of the force."
    out["reasons"] = reasons
    return out


def _check_hand_placed(e, taken):
    """Raise ValueError unless e is a well-formed hand-placed actuator."""
    who = e.get("id", "(no id)") if isinstance(e, dict) else "(not an object)"
    if not isinstance(e, dict):
        raise ValueError(f"hand-placed entry {who} is not an object")
    if "record_file" in e or "record_sha256" in e:
        raise ValueError(f"hand-placed entry {who} names a record file; "
                         f"hand-placed entries never come from data/records")
    basis = e.get("basis")
    if not isinstance(basis, str) or not basis.strip():
        raise ValueError(f"hand-placed entry {who} has no 'basis' saying "
                         f"where its placement came from")
    need = [k for k in ACTUATOR_KEYS
            if k not in ("record_file", "record_sha256", "hand_placed",
                         "length", "bend_check", "tasks", "force",
                         "force_withheld") + SHOWN_KEYS]
    missing = [k for k in need if k not in e]
    if missing:
        raise ValueError(f"hand-placed entry {who} is missing {missing}")
    if e.get("hand_placed", True) is not True:
        raise ValueError(f"hand-placed entry {who} says hand_placed is not true")
    if e["id"] in taken:
        raise ValueError(f"hand-placed entry {who} has the same id as an "
                         f"actuator already in the map")
    if e["surface"] is not None:
        raise ValueError(f"hand-placed entry {who} has a surface; a surface "
                         f"is predicted from a record's rows, so a "
                         f"hand-placed entry carries \"surface\": null")
    allowed = {"length": (None, NO_LENGTH), "bend_check": (None,)}
    for k, ok in allowed.items():
        if e.get(k) not in ok:
            raise ValueError(f"hand-placed entry {who} gives '{k}'; a "
                             f"hand-placed entry has no body length, so it "
                             f"carries none")
    for k in ("tasks", "force", "force_withheld"):
        if e.get(k) is not None:
            raise ValueError(f"hand-placed entry {who} gives '{k}'; that is "
                             f"computed from a record, so a hand-placed "
                             f"entry carries none")
    if not isinstance(e["legs"], list) or not e["legs"]:
        raise ValueError(f"hand-placed entry {who} has no legs")
    if any(isinstance(leg, dict) and leg.get("length_sensitivity") is not None
           for leg in e["legs"]):
        raise ValueError(f"hand-placed entry {who} gives a leg "
                         f"'length_sensitivity'; a hand-placed entry has no "
                         f"body length, so it carries none")
    # the quantities that need a body length may be left out, or null
    required = [k for k in VALUE_KEYS if k not in LENGTH_KEYS]
    for leg in e["legs"]:
        miss = [k for k in LEG_KEYS
                if k not in leg and k != "length_sensitivity"]
        if miss:
            raise ValueError(f"hand-placed entry {who}: a leg is missing {miss}")
        n = len(leg["input"].get("values") or [])
        held = set(leg["values"])
        if not set(required) <= held or not held <= set(VALUE_KEYS):
            raise ValueError(f"hand-placed entry {who}: leg values must hold "
                             f"{required}, and may also hold {LENGTH_KEYS}, "
                             f"nothing else")
        for k in VALUE_KEYS:
            v = leg["values"].get(k)
            if v is None and (k == "twist_rad" or k in LENGTH_KEYS):
                continue
            if not isinstance(v, list) or len(v) != n:
                raise ValueError(f"hand-placed entry {who}: leg value {k} "
                                 f"must be a list as long as the inputs ({n})")


def load_hand_placed(path, taken=()):
    """The hand-placed actuators in `path`, checked, each with
    "hand_placed": true. A hand-placed entry has no body length: "length"
    comes out all null, "bend_check" and each leg's "length_sensitivity"
    null (an entry giving anything else there is refused), and each value
    that needs a length is null unless the file gives it; none is ever
    computed. Raises ValueError on a malformed entry."""
    with open(path, encoding="utf-8") as fo:
        doc = json.load(fo)
    out, taken = [], set(taken)
    for e in doc.get("actuators") or []:
        _check_hand_placed(e, taken)
        taken.add(e["id"])
        legs = [{**leg, "values": {**leg["values"],
                                   **{k: None for k in LENGTH_KEYS
                                      if k not in leg["values"]}},
                 "length_sensitivity": leg.get("length_sensitivity")}
                for leg in e["legs"]]
        out.append({**e, "hand_placed": True, "legs": legs,
                    "length": dict(NO_LENGTH), "bend_check": None,
                    "tasks": None, "force": None, "force_withheld": None})
    return out


def load_task_cones(path):
    """The declared task cones in `path`, checked. Raises ValueError on a
    malformed entry. Each holds "task" (a task file id), "n", "mount",
    "yaw_deg" (the placement the needed direction is worked out for),
    "half_angle_deg" (above 0, below 90) with "half_angle_basis", and
    "least_travel_m" (a positive number, or null with "least_travel_note"
    saying why there is none). "push_basis" (words) and
    "push_threshold_ratio" (a positive number) come together or not at
    all; where they are absent the cone carries no needed push. "stroke"
    is optional: where given it holds "angle_deg" (above 0, at most
    STROKE_MAX_DEG, the most the source script accepts), "source" (where
    the angle is read from) and "basis" (words); it is None where absent."""
    with open(path, encoding="utf-8") as fo:
        doc = json.load(fo)
    out, seen = [], set()
    for c in doc.get("cones") or []:
        who = c.get("task") if isinstance(c, dict) else None
        if not isinstance(who, str) or not who:
            raise ValueError("a task cone has no 'task'")
        if who in seen:
            raise ValueError(f"task cone {who} is declared twice")
        seen.add(who)
        if isinstance(c.get("n"), bool) or not isinstance(c.get("n"), int) \
                or c["n"] < 1:
            raise ValueError(f"task cone {who}: 'n' must be a whole number")
        if not isinstance(c.get("mount"), str):
            raise ValueError(f"task cone {who}: 'mount' must be a mount style")
        for k in ("yaw_deg", "half_angle_deg"):
            if isinstance(c.get(k), bool) or not isinstance(c.get(k), (int, float)):
                raise ValueError(f"task cone {who}: '{k}' must be a number")
        if not 0.0 < c["half_angle_deg"] < 90.0:
            raise ValueError(f"task cone {who}: 'half_angle_deg' must lie "
                             f"between 0 and 90")
        basis = c.get("half_angle_basis")
        if not isinstance(basis, str) or not basis.strip():
            raise ValueError(f"task cone {who} has no 'half_angle_basis' "
                             f"saying where the angle comes from")
        least = c.get("least_travel_m")
        if least is None:
            note = c.get("least_travel_note")
            if not isinstance(note, str) or not note.strip():
                raise ValueError(f"task cone {who} has no least travel and "
                                 f"no 'least_travel_note' saying why")
        elif not _positive_number(least):
            raise ValueError(f"task cone {who}: 'least_travel_m' must be a "
                             f"positive number or null")
        push_basis = c.get("push_basis")
        ratio = c.get("push_threshold_ratio")
        if (push_basis is None) != (ratio is None):
            raise ValueError(f"task cone {who}: 'push_basis' and "
                             f"'push_threshold_ratio' come together or not "
                             f"at all")
        if push_basis is not None:
            if not isinstance(push_basis, str) or not push_basis.strip():
                raise ValueError(f"task cone {who}: 'push_basis' must say "
                                 f"how the needed push is worked out")
            if not _positive_number(ratio):
                raise ValueError(f"task cone {who}: 'push_threshold_ratio' "
                                 f"must be a positive number")
            ratio = float(ratio)
        stroke = c.get("stroke")
        if stroke is not None:
            if not isinstance(stroke, dict):
                raise ValueError(f"task cone {who}: 'stroke' must be an object")
            ang = stroke.get("angle_deg")
            if isinstance(ang, bool) or not isinstance(ang, (int, float)) \
                    or not 0.0 < ang <= STROKE_MAX_DEG:
                raise ValueError(f"task cone {who}: 'stroke.angle_deg' must "
                                 f"be above 0 and at most {STROKE_MAX_DEG:g}")
            src = stroke.get("source")
            if not isinstance(src, dict) or not all(
                    isinstance(src.get(k), str) and src[k].strip()
                    for k in STROKE_SOURCE_KEYS) \
                    or not isinstance(src.get("dirty"), bool):
                raise ValueError(f"task cone {who}: 'stroke.source' must pin "
                                 f"{', '.join(STROKE_SOURCE_KEYS)} and 'dirty'")
            sb = stroke.get("basis")
            if not isinstance(sb, str) or not sb.strip():
                raise ValueError(f"task cone {who}: 'stroke' has no 'basis' "
                                 f"saying what the stroke means")
            stroke = {"angle_deg": float(ang), "source": dict(src), "basis": sb}
        back = c.get("return_clear")
        if back is not None:
            if stroke is None:
                raise ValueError(f"task cone {who}: 'return_clear' needs a "
                                 f"'stroke' to come back from")
            extra = (sorted(set(back) - set(RETURN_KEYS))
                     if isinstance(back, dict) else None)
            if extra is None or sorted(set(back) - set(extra)) \
                    != sorted(RETURN_KEYS) \
                    or extra not in ([], sorted(RETURN_OPTIONAL_KEYS)):
                raise ValueError(f"task cone {who}: 'return_clear' must hold "
                                 f"exactly the keys {list(RETURN_KEYS)}"
                                 f", with or without both of "
                                 f"{list(RETURN_OPTIONAL_KEYS)}")
            if extra:
                if back["stroke_from"] != STROKE_FROM_OWN_START:
                    raise ValueError(f"task cone {who}: 'return_clear."
                                     f"stroke_from' may only be "
                                     f"{STROKE_FROM_OWN_START!r}")
                if not isinstance(back["stroke_from_basis"], str) \
                        or not back["stroke_from_basis"].strip():
                    raise ValueError(f"task cone {who}: 'return_clear' has "
                                     f"no 'stroke_from_basis'")
            for k in ("basis", "drawn_depth_basis"):
                if not isinstance(back[k], str) or not back[k].strip():
                    raise ValueError(f"task cone {who}: 'return_clear' has "
                                     f"no '{k}'")
            if not _positive_number(back["drawn_depth_m"]):
                raise ValueError(f"task cone {who}: 'return_clear."
                                 f"drawn_depth_m' must be a positive number")
            back = {"basis": back["basis"],
                    "drawn_depth_m": float(back["drawn_depth_m"]),
                    "drawn_depth_basis": back["drawn_depth_basis"],
                    "stations": RETURN_STATIONS,
                    **{k: back[k] for k in extra}}
        out.append({"task": who, "n": c["n"], "mount": c["mount"],
                    "yaw_deg": float(c["yaw_deg"]),
                    "half_angle_deg": float(c["half_angle_deg"]),
                    "half_angle_basis": basis,
                    "least_travel_m": None if least is None else float(least),
                    "least_travel_note": c.get("least_travel_note"),
                    "push_basis": push_basis,
                    "push_threshold_ratio": ratio,
                    "stroke": stroke,
                    "return_clear": back})
    return out


def angle_deg(v, e):
    """Angle in degrees (0 to 180) between v and e; None where either has
    no length."""
    v, e = np.asarray(v, float), np.asarray(e, float)
    nv, ne = float(np.linalg.norm(v)), float(np.linalg.norm(e))
    if nv == 0.0 or ne == 0.0:
        return None
    return float(np.degrees(np.arccos(np.clip(float(v @ e) / (nv * ne),
                                              -1.0, 1.0))))


def _runs(inside, inputs):
    """[[first input, last input], ...] over each unbroken run of levels
    that are inside (measured levels only; nothing between levels)."""
    runs, start = [], None
    for k, on in enumerate(list(inside) + [False]):
        if on and start is None:
            start = k
        elif not on and start is not None:
            runs.append([float(inputs[start]), float(inputs[k - 1])])
            start = None
    return runs


def stroke_travel_m(stroke, task):
    """The tip travel for one stroke: the stroke angle (radians) times the
    contact ring's distance from the turn axis, the ball radius times the
    cosine of the ring altitude, both from the task file. The inverse of
    gramian_workshop.arrange.Arrangement.turn_per_stroke_deg."""
    return float(np.radians(stroke["angle_deg"])
                 * float(task["object"]["radius_m"])
                 * np.cos(np.radians(float(task["contact_ring"]["altitude_deg"]))))


def stroke_of(legs, angles_by_leg, e, half, travel):
    """How far each leg carries the tip along the needed direction e,
    against one stroke's travel. Per leg: "along_m" (the tip position from
    rest dotted with e, at each level), "across_m" (at each level, the size
    of the part of the tip position from rest at right angles to e: never
    negative, null where "along_m" or the position is null; with "along_m"
    it gives the angle to e), "reaches_end" (some level is at or
    beyond the travel), "end_input" (the leg's input where "along_m" first
    reaches the travel, in a straight line between the two logged levels
    either side; None if never reached), "end_input_is" (says so; where the
    first logged level is already at or beyond the end there is no level
    below to interpolate from, so the end input is that level's own and
    "end_input_is" says that instead), "inside_to_end" (it reaches the end and every level from
    the leg's start up to and including the first level at or beyond the
    end is within the half-angle; a level with no direction counts as
    inside) and "end_angle_deg" (the angle at that first level)."""
    out_legs, furthest = [], None
    for leg, angles in zip(legs, angles_by_leg):
        v, u = leg["values"], leg["input"]["values"]
        along = [float(np.dot([x, y, z], e))
                 for x, y, z in zip(v["axial_m"], v["side_y_m"], v["side_z_m"])]
        # The size of the part at right angles to e, from the same vector and
        # the same unit direction as along: |p x e|, never negative, so that
        # atan2(across, along) is the angle between p and e.
        across = [None if s is None or None in (x, y, z) else
                  float(np.linalg.norm(np.cross([x, y, z], e)))
                  for s, x, y, z in zip(along, v["axial_m"], v["side_y_m"],
                                        v["side_z_m"])]
        if along:
            furthest = max(along) if furthest is None else max(furthest, *along)
        k = next((i for i, s in enumerate(along) if s >= travel), None)
        end_input = end_angle = None
        end_is = STROKE_END_INPUT_IS
        inside_to_end = False
        if k is not None:
            if k == 0:
                end_input, end_is = float(u[0]), STROKE_FIRST_LEVEL_IS
            else:
                f = (travel - along[k - 1]) / (along[k] - along[k - 1])
                end_input = float(u[k - 1] + f * (u[k] - u[k - 1]))
            end_angle = angles[k]
            inside_to_end = all(a is None or a <= half for a in angles[:k + 1])
        out_legs.append({"along_m": along, "across_m": across,
                         "reaches_end": k is not None,
                         "end_input": end_input,
                         "end_input_is": end_is,
                         "inside_to_end": bool(inside_to_end),
                         "end_angle_deg": end_angle})
    return {"reaches_end": any(g["reaches_end"] for g in out_legs),
            "inside_to_end": any(g["inside_to_end"] for g in out_legs),
            "furthest_along_m": furthest,
            "legs": out_legs}


def _crossings(along, normal, s):
    """Travel along the contact normal wherever a line through the given
    points (in order, straight between neighbours) is at travel `s` along
    the stroke. Points holding None are skipped over."""
    out = []
    for a0, a1, n0, n1 in zip(along, along[1:], normal, normal[1:]):
        if None in (a0, a1, n0, n1):
            continue
        if a0 == a1:
            if a0 == s:
                out += [n0, n1]
            continue
        f = (s - a0) / (a1 - a0)
        if 0.0 <= f <= 1.0:
            out.append(n0 + f * (n1 - n0))
    return out


def return_clear_of(legs, surface, t, decl, travel):
    """The second check for one task entry `t` (from task_of, with its
    stroke): see the module docstring. Returns "stroke_leg" (which leg is
    the stroke path, or None with "reason" and "passes" None),
    "positions_read_from", "stations_along_m", "outward_reach_m" and
    "inward_reach_m" (one per station, never negative),
    "least_outward_reach_m", "furthest_inward_reach_m" and "passes"."""
    k_leg = next((i for i, g in enumerate(t["stroke"]["legs"])
                  if g["inside_to_end"]), None)
    if k_leg is None:
        return {"stroke_leg": None, "passes": None, "reason": NO_STROKE_PATH}
    e, nn = np.asarray(t["needed_direction"]), np.asarray(t["contact_normal"])

    def split(xs, ys, zs):
        al, no = [], []
        for x, y, z in zip(xs, ys, zs):
            ok = None not in (x, y, z)
            al.append(float(np.dot([x, y, z], e)) if ok else None)
            no.append(float(np.dot([x, y, z], nn)) if ok else None)
        return al, no

    # every leg starts from rest, the origin of the tip positions, whether
    # or not its first logged level is the rest level
    lines = [split([0.0] + list(g["values"]["axial_m"]),
                   [0.0] + list(g["values"]["side_y_m"]),
                   [0.0] + list(g["values"]["side_z_m"])) for g in legs]
    if surface is not None:
        v = surface["values"]
        X, Y, Z = v["axial_m"], v["side_y_m"], v["side_z_m"]
        for i in range(len(X)):
            lines.append(split(X[i], Y[i], Z[i]))
        for j in range(len(X[0])):
            lines.append(split([r[j] for r in X], [r[j] for r in Y],
                               [r[j] for r in Z]))
    al, no = lines[k_leg]
    end = next(i for i, a in enumerate(al) if a is not None and a >= travel)
    stations = [travel * (j + 1) / RETURN_STATIONS
                for j in range(RETURN_STATIONS)]
    stations[-1] = min(stations[-1], al[end])
    outward, inward = [], []
    for s in stations:
        ref = _crossings(al[:end + 1], no[:end + 1], s)[0]
        there = [c for a, n in lines for c in _crossings(a, n, s)]
        outward.append(max(ref - min(there), 0.0))
        inward.append(max(max(there) - ref, 0.0))
    return {"stroke_leg": k_leg,
            "positions_read_from": ("its legs" if surface is None else
                                    "its legs and its surface over both "
                                    "inputs"),
            "stations_along_m": stations,
            "outward_reach_m": outward, "inward_reach_m": inward,
            "least_outward_reach_m": min(outward),
            "furthest_inward_reach_m": max(inward),
            "passes": bool(min(outward) >= decl["drawn_depth_m"])}


def _points_at(P, along, s):
    """The points where a line through the points P (in order, straight
    between neighbours) is at travel `s` along the stroke; `along` is each
    point's travel."""
    out = []
    for p0, p1, a0, a1 in zip(P, P[1:], along, along[1:]):
        if a0 == a1:
            if a0 == s:
                out += [p0, p1]
            continue
        f = (s - a0) / (a1 - a0)
        if 0.0 <= f <= 1.0:
            out.append(p0 + f * (p1 - p0))
    return out


def return_clear_own_start_of(legs, surface, t, decl, travel, half):
    """The second check for one task entry `t` where the cone declares
    "stroke_from": "own_start": see the module docstring. Returns
    "stroke_from", "stroke_line" and "return_line" (each: "what", the
    swept and held inputs, and "points_m", the line's position at the
    stroke's start and at each station), "stroke_start_m" (the stroke
    line's first point, from rest) with its travel along the needed
    direction and into the object, "stroke_worst_angle_deg" (from its own
    start), "stroke_end_input", "stations_along_m" (from rest),
    "outward_reach_m" (one per station), "least_outward_reach_m" and
    "passes"; or "passes" None with "reason" where no line can be the
    stroke path."""
    e, nn = np.asarray(t["needed_direction"]), np.asarray(t["contact_normal"])
    lines = []      # (description, points, swept input levels)
    for i, g in enumerate(legs):
        v, u = g["values"], g["input"]["values"]
        rows = [(None, (0.0, 0.0, 0.0))] + list(zip(
            u, zip(v["axial_m"], v["side_y_m"], v["side_z_m"])))
        rows = [(ui, p) for ui, p in rows if None not in p]
        lines.append(({"what": "leg", "leg": i, "label": g["label"],
                       "swept_input": g["input"].get("name"),
                       "held_input": None, "held_at": None},
                      np.array([p for _, p in rows], float),
                      [ui for ui, _ in rows]))
    if surface is not None:
        v, ins = surface["values"], surface["inputs"]
        G = np.stack([np.array(v[k], float) for k in
                      ("axial_m", "side_y_m", "side_z_m")], -1)
        for i, level in enumerate(ins[0]["values"]):
            lines.append(({"what": "surface line", "swept_input": ins[1]["name"],
                           "held_input": ins[0]["name"],
                           "held_at": float(level),
                           "held_unit": ins[0].get("unit")},
                          G[i], list(ins[1]["values"])))
        for j, level in enumerate(ins[1]["values"]):
            lines.append(({"what": "surface line", "swept_input": ins[0]["name"],
                           "held_input": ins[1]["name"],
                           "held_at": float(level),
                           "held_unit": ins[1].get("unit")},
                          G[:, j], list(ins[0]["values"])))
    best = None
    for about, P, U in lines:
        if not np.all(np.isfinite(P)):
            continue
        Q = P - P[0]
        k = next((i for i, a in enumerate(Q @ e) if a >= travel), None)
        if not k:
            continue
        angles = [angle_deg(q, e) for q in Q[1:k + 1]]
        angles = [a for a in angles if a is not None]
        if not angles or max(angles) > half:
            continue
        A = P @ e
        stations = [float(A[0] + travel * (j + 1) / RETURN_STATIONS)
                    for j in range(RETURN_STATIONS)]
        stations[-1] = min(stations[-1], float(A[k]))
        ref = [P[0]] + [_points_at(P[:k + 1], A[:k + 1], s)[0]
                        for s in stations]
        f = (travel - (A[k - 1] - A[0])) / (A[k] - A[k - 1])
        end_input = (None if U[k - 1] is None
                     else float(U[k - 1] + f * (U[k] - U[k - 1])))
        for about_r, R, _ in lines:
            if not np.all(np.isfinite(R)):
                continue
            back = []
            for s in [float(A[0])] + stations:
                there = _points_at(R, R @ e, s)
                if not there:
                    break
                back.append(max(there, key=lambda p: float(p @ nn)))
            if len(back) != len(ref):
                continue
            gaps = [float((p - r) @ nn) for p, r in zip(ref[1:], back[1:])]
            if best is None or min(gaps) > best["least"]:
                best = {"least": min(gaps), "gaps": gaps, "stroke": about,
                        "ref": ref, "back": back, "return": about_r,
                        "stations": stations, "start": P[0],
                        "angle": max(angles), "end_input": end_input}
    if best is None:
        return {"stroke_from": STROKE_FROM_OWN_START, "stroke_line": None,
                "passes": None, "reason": NO_STROKE_LINE}
    pts = lambda L: [[float(c) for c in p] for p in L]
    least = max(best["least"], 0.0)
    return {"stroke_from": STROKE_FROM_OWN_START,
            "stroke_line": {**best["stroke"], "points_m": pts(best["ref"])},
            "return_line": {**best["return"], "points_m": pts(best["back"])},
            "stroke_start_m": [float(c) for c in best["start"]],
            "stroke_start_along_m": float(best["start"] @ e),
            "stroke_start_into_object_m": float(best["start"] @ nn),
            "stroke_worst_angle_deg": float(best["angle"]),
            "stroke_end_input": best["end_input"],
            "stations_along_m": best["stations"],
            "outward_reach_m": [max(g, 0.0) for g in best["gaps"]],
            "least_outward_reach_m": least,
            "passes": bool(least >= decl["drawn_depth_m"])}


def task_of(rec, quality, legs, cone, task):
    """One actuator against one declared task cone. The needed direction is
    the one the backwards lookup uses (lookup.directions), in the record's
    own frame, for the cone's placement. Each level of each leg is tested
    on the tip position from rest: inside when the angle between it and the
    needed direction is no more than the half-angle (and, where the cone
    declares a least travel, the travel along the needed direction is at
    least that). A level with the tip at the origin has no direction: its
    angle and its inside are null. Where the direction cannot be worked out
    the entry says why and holds nothing else."""
    try:
        e, nn, _ = directions(Finger(rec, quality), task, cone["n"],
                              cone["mount"], cone["yaw_deg"])
        e, nn = np.asarray(e, float), np.asarray(nn, float)
        if not (np.all(np.isfinite(e)) and np.all(np.isfinite(nn))):
            raise ValueError("the needed direction is not a number")
    except Exception as ex:  # noqa: BLE001 - any record that cannot be mounted
        return {"task": cone["task"], "reason": f"no needed direction for "
                f"this record ({type(ex).__name__}: {ex})"}
    half, least = cone["half_angle_deg"], cone["least_travel_m"]
    out_legs, angles_all, angles_by_leg = [], [], []
    for leg in legs:
        v = leg["values"]
        angles, inside = [], []
        for x, y, z in zip(v["axial_m"], v["side_y_m"], v["side_z_m"]):
            a = angle_deg([x, y, z], e)
            angles.append(a)
            if a is None:
                inside.append(None)
                continue
            far = least is None or float(np.dot([x, y, z], e)) >= least
            inside.append(bool(a <= half and far))
        angles_all += [a for a in angles if a is not None]
        angles_by_leg.append(angles)
        out_legs.append({"angle_deg": angles, "inside": inside,
                         "inside_input": _runs([i is True for i in inside],
                                               leg["input"]["values"])})
    out = {"task": cone["task"],
           "needed_direction": [float(c) for c in e],
           "contact_normal": [float(c) for c in nn],
           "least_angle_deg": min(angles_all) if angles_all else None,
           "enters": any(i is True for g in out_legs for i in g["inside"]),
           "legs": out_legs}
    if cone.get("stroke") is not None:
        out["stroke"] = stroke_of(legs, angles_by_leg, e, half,
                                  stroke_travel_m(cone["stroke"], task))
    return out


def _column(block, key, n, width=None):
    """block[key] as a list of n numbers (or n lists of `width` numbers),
    or None where the record does not hold it in that shape."""
    v = block.get(key)
    if not isinstance(v, list) or len(v) != n:
        return None
    try:
        a = np.asarray(v, float)
    except (TypeError, ValueError):
        return None
    want = (n,) if width is None else (n, width)
    if a.shape != want or not np.all(np.isfinite(a)):
        return None
    return a.tolist()


def force_of(rec, task_entries):
    """The record's blocked block, as it stands, with two angles worked out
    from it: the angle of the force off the base-to-tip axis, and, where
    the whole vector is held, its angle to each task's needed direction
    and contact normal. None where the record holds no force with the tip
    held."""
    b = rec.get("blocked") or {}
    U = b.get("u")
    if not isinstance(U, list) or not U:
        return None
    n = len(U)
    axial = _column(b, "force_axial_N", n)
    side = _column(b, "force_transverse_N", n)
    vec = _column(b, "force_N", n, 3)
    if axial is None and side is None and vec is None:
        return None
    if vec is not None:
        size = [float(np.linalg.norm(f)) for f in vec]
        size_is = "whole force"
        off_axis = [angle_deg(f, [1.0, 0.0, 0.0]) for f in vec]
    elif axial is not None and side is not None:
        size = [float(np.hypot(a, t)) for a, t in zip(axial, side)]
        size_is = "whole force"
        off_axis = [None if a == 0.0 and t == 0.0
                    else float(np.degrees(np.arctan2(abs(t), a)))
                    for a, t in zip(axial, side)]
    else:
        size = [abs(float(f)) for f in (axial if axial is not None else side)]
        size_is = ("axial part only" if axial is not None
                   else "sideways part only")
        off_axis = None
    to_task = []
    for t in task_entries or []:
        if vec is None or "needed_direction" not in t:
            continue
        to_task.append({
            "task": t["task"],
            "to_needed_direction_deg": [angle_deg(f, t["needed_direction"])
                                        for f in vec],
            "to_contact_normal_deg": [angle_deg(f, t["contact_normal"])
                                      for f in vec]})
    chans = _channels(rec)
    return {"method": b.get("method"),
            "moment_method": b.get("moment_method"),
            "input": {"names": [c.get("name") for c in chans],
                      "units": [c.get("unit") for c in chans],
                      "values": [[float(x) for x in
                                  (row if isinstance(row, list) else [row])]
                                 for row in U]},
            "axial_N": axial, "sideways_size_N": side, "vector_N": vec,
            "direction_known": vec is not None,
            "size_N": size, "size_is": size_is, "off_axis_deg": off_axis,
            "moment_Nm": _column(b, "moment_Nm", n, 3),
            "twist_tip_Nm": _column(b, "twist_tip_Nm", n),
            "to_task": to_task}


class _UnitPush:
    """A finger of unit force that pushes along its own +X and does not
    move; the aimed mount points that push along the contact normal. Used
    for the task-level needed push, which then depends on no record."""
    b = np.array([1.0, 0.0, 0.0])
    d = None
    F = 1.0


_HOLD_MEMO = {}


def _hold(P, Nn, cup, B):
    """soft_hold with a margin for one contact set, unit force per finger,
    remembered on the exact inputs (the tests build the document many
    times)."""
    key = (np.asarray(P, float).tobytes(), np.asarray(Nn, float).tobytes(),
           np.asarray(cup.com, float).tobytes(),
           np.asarray(cup.weight_demand(), float).tobytes(),
           float(cup.mu), float(cup.L),
           tuple(None if b is None else np.asarray(b, float).tobytes()
                 for b in B))
    if key not in _HOLD_MEMO:
        s = soft_hold(P, Nn, cup.com, cup.weight_demand(), cup.mu, cup.L, B,
                      [1.0] * len(B), None, want_margin=True)
        _HOLD_MEMO[key] = (s["reason"], s["margin"])
    return _HOLD_MEMO[key]


def needed_push(finger, task, n, mount, yaw_deg, duty):
    """The push each finger must make along its own push direction for the
    hand to hold the task's object, over the contact sets the duty
    schedule makes: {"needed_N", "reason", "worst_contact_count"}. The
    finger's force is taken as 1 N, so the needed push is 1 over the
    smallest margin. Null with the reason where some contact set has no
    margin (it cannot hold the object in any amount), or where the margin
    reaches the search's cap (then 1 over it would not be the needed
    push)."""
    arr = Arrangement(finger, task, n, mount, yaw_deg)
    worst, seen = None, set()
    for row in schedule(n, duty):
        k = row.tobytes()
        if k in seen:
            continue
        seen.add(k)
        P, Nn, B, _, _ = arr.subset(row)
        reason, margin = _hold(P, Nn, arr.cup, B)
        count = int(row.sum())
        if reason in NO_MARGIN:
            return {"needed_N": None, "reason": reason,
                    "worst_contact_count": count}
        if margin is None or not np.isfinite(margin) or margin <= 0.0:
            return {"needed_N": None, "reason": "no margin",
                    "worst_contact_count": count}
        if margin >= SCALE_MAX:
            return {"needed_N": None, "reason": "margin at the search cap",
                    "worst_contact_count": count}
        if worst is None or margin < worst[0]:
            worst = (float(margin), count)
    if worst is None:
        return {"needed_N": None, "reason": "no_contact",
                "worst_contact_count": 0}
    return {"needed_N": 1.0 / worst[0], "reason": None,
            "worst_contact_count": worst[1]}


def _level_runs(on, rows):
    """Runs of levels where `on` holds, in the format of inside_input: for
    one input, [[first, last], ...] of input values; for several inputs,
    each end is the whole row of input values, in stored order."""
    if all(len(r) == 1 for r in rows):
        return _runs(on, [r[0] for r in rows])
    runs, start = [], None
    for k, x in enumerate(list(on) + [False]):
        if x and start is None:
            start = k
        elif not x and start is not None:
            runs.append([[float(v) for v in rows[start]],
                         [float(v) for v in rows[k - 1]]])
            start = None
    return runs


def available_push(rule, b, F, force):
    """The held-tip force along the push direction b at each level, by the
    rule that gave b: force_N, the stored vector; axial_transverse, the
    axial part and the size of the sideways part, sideways taken along +Y
    as the push direction takes it; polarity, the stored scalar only, one
    level. None where it cannot be had."""
    if b is None:
        return None
    b = np.asarray(b, float)
    if rule == "force_N" and force and force["vector_N"] is not None:
        return [float(np.dot(f, b)) for f in force["vector_N"]]
    if rule == "axial_transverse" and force and force["axial_N"] is not None:
        side = force["sideways_size_N"] or [0.0] * len(force["axial_N"])
        return [float(np.dot([a, abs(t), 0.0], b))
                for a, t in zip(force["axial_N"], side)]
    if rule == "polarity" and F is not None:
        return [float(F)]
    return None


def push_of(rec, quality, legs, force, entry, cone, task, task_needed):
    """One actuator's push against one task: the needed push for its own
    push direction (unit force, so the record's force never meets the
    margin's cap), what it can push along that direction at each held-tip
    level, and where that meets the need and the cone together."""
    finger = Finger(rec, quality)
    rule, b, F = finger.b_rule, finger.b, finger.F
    if force is None and rule != "polarity":
        return None
    finger.F = 1.0
    duty = float(task["wave"]["duty"])
    need = needed_push(finger, task, cone["n"], cone["mount"],
                       cone["yaw_deg"], duty)
    needed = need["needed_N"]
    thr = cone["push_threshold_ratio"]
    avail = available_push(rule, b, F, force)
    rows = force["input"]["values"] if force else None
    if rule == "polarity" and (force is None or len(avail) != len(rows)):
        rows = None
    ratio = (None if avail is None or needed is None
             else [a / needed for a in avail])
    size_only = None
    if b is None and force is not None and task_needed:
        size_only = [s / task_needed for s in force["size_N"]]
    meets = None
    if ratio is not None and rows is not None:
        meets = _level_runs([r >= thr for r in ratio], rows)
    both, why = None, None
    names = force["input"]["names"] if force else []
    if len(names) > 1:
        why = MULTI_INPUT_BOTH
    elif b is None:
        why = ("Not tested: the direction of the held-tip force is not "
               "stored, so the push along the placed direction is not known.")
    elif needed is None:
        why = f"Not tested: the needed push has no value ({need['reason']})."
    elif meets is None:
        why = "Not tested: the push is not stored at named input levels."
    else:
        legs = [(leg, g) for leg, g in zip(legs, entry["legs"])
                if leg["input"]["name"] == names[0]
                and leg["input"]["unit"] == force["input"]["units"][0]]
        if not legs:
            why = ("Not tested: no free-path leg is on the same input, in "
                   "the same unit, as the held-tip levels.")
        else:
            runs = [r for _, g in legs for r in g["inside_input"]]
            on = [r >= thr and any(lo <= row[0] <= hi for lo, hi in runs)
                  for r, row in zip(ratio, rows)]
            both = _level_runs(on, rows)
    return {"needed_N": needed, "reason": need["reason"],
            "worst_contact_count": need["worst_contact_count"],
            "input": force["input"] if force else None,
            "available_N": avail, "ratio": ratio,
            "size_only_ratio": size_only, "meets_input": meets,
            "both_input": both, "both_reason": why,
            "direction_known": b is not None, "rule": rule}


def task_push(cone, task, passers_from):
    """The task-level needed push for a cone's placement, with a finger of
    unit force pushing along the contact normal; also at other finger
    counts and at the task file's other duty values. None where the cone
    declares no push."""
    if cone["push_basis"] is None:
        return None
    duty = float(task["wave"]["duty"])
    at = needed_push(_UnitPush(), task, cone["n"], cone["mount"],
                     cone["yaw_deg"], duty)
    by_n = [{"n": n, **needed_push(_UnitPush(), task, n, cone["mount"],
                                   cone["yaw_deg"], duty)}
            for n in PUSH_NS]
    by_duty = [{"duty": float(d),
                **needed_push(_UnitPush(), task, cone["n"], cone["mount"],
                              cone["yaw_deg"], float(d))}
               for d in (task["wave"].get("duty_sensitivity") or [])]
    return {"needed_N": at["needed_N"], "reason": at["reason"],
            "duty": duty, "worst_contact_count": at["worst_contact_count"],
            "threshold_ratio": cone["push_threshold_ratio"],
            "by_n": by_n, "by_duty": by_duty,
            "passers_hold_range_N": passers_from,
            "basis": cone["push_basis"]}


def passers_range(actuators, task, compared):
    """Over the clean records (those the stored lookup compared that are on
    the map and have a push), the open interval between the largest push
    among those that never meet the need and the smallest among those that
    do, each record's push taken at its best held-tip level. Any needed
    push inside it splits the records the same way. None where either side
    is empty."""
    fail, ok = [], []
    for a in actuators:
        if a["id"] not in compared:
            continue
        t = next((t for t in a["tasks"] or [] if t["task"] == task), None)
        p = (t or {}).get("push")
        if not p or p["available_N"] is None or p["meets_input"] is None:
            continue
        (ok if p["meets_input"] else fail).append(max(p["available_N"]))
    if not fail or not ok:
        return None
    return [max(fail), min(ok)]


def drawn_direction(actuators, task):
    """The one direction a task's cone is drawn about: the component-wise
    median of the needed directions of the actuators that have one, made
    unit length. Each actuator is tested against its own direction, not
    this one. None where no actuator has a direction."""
    dirs = [t["needed_direction"] for a in actuators for t in a["tasks"] or []
            if t["task"] == task and "needed_direction" in t]
    if not dirs:
        return None
    m = np.median(np.asarray(dirs, float), axis=0)
    return (m / np.linalg.norm(m)).tolist()


def build(drafts=False, hand_placed=HAND_PLACED, caveats=CAVEATS,
          rules=TWO_INPUT_RULES, extra_checks=EXTRA_CHECKS, lengths=LENGTHS, task_cones=TASK_CONES,
          stiffness=STIFFNESS, pressure_scale=PRESSURE_SCALE,
          force_withheld=FORCE_WITHHELD, names=NAMES,
          thumbnails=THUMBNAILS, site_dir=SITE_DIR):
    """The whole document, as a dict."""
    with open(caveats, encoding="utf-8") as fo:
        cav = json.load(fo).get("records") or {}
    from backwards_lookup import draft_records
    drafts_found = draft_records()
    items = []                 # (record file path, record, flags, draft)
    for rid, e in load_manifest()["records"].items():
        items.append((os.path.join(ROOT, "data", "records", e["file"]),
                      load_record(rid), e.get("quality"), False))
    # every id, drafts too, so a declaration for a draft is checked even
    # when drafts are left out of this document
    own_lengths = {rec["id"]: record_length(rec) for _, rec, _, _ in items}
    all_records = {rec["id"]: rec for _, rec, _, _ in items}
    record_paths = {rec["id"]: p for p, rec, _, _ in items}
    for path in drafts_found:
        rec = load_record(path)
        own_lengths[rec["id"]] = record_length(rec)
        all_records[rec["id"]] = rec
        record_paths[rec["id"]] = path
    known = set(own_lengths)
    declared = load_two_input_rules(rules, known)
    # an entry for a record with no declared rule has no surface to check
    extras = load_extra_checks(extra_checks, known)
    estimates = load_lengths(lengths, own_lengths)
    moduli = load_stiffness(stiffness, {rid: record_modulus(r)
                                        for rid, r in all_records.items()})
    corrections, correction_docs = load_pressure_scale(pressure_scale,
                                                       all_records)
    withheld = load_force_withheld(force_withheld, record_paths)
    shown_as = load_names(names, {
        rid: (r.get("source") or {}).get("kind")
        for rid, r in all_records.items()})
    pictures = load_thumbnails(thumbnails, known, site_dir)
    cones = load_task_cones(task_cones)
    task_files = {c["task"]: load_task(c["task"]) for c in cones}
    for c in cones:
        if c["stroke"] is not None:
            c["stroke"] = {**c["stroke"], "travel_m": stroke_travel_m(
                c["stroke"], task_files[c["task"]])}
    pushes = {c["task"]: task_push(c, task_files[c["task"]], None)
              for c in cones}
    if drafts:
        for path, q in drafts_found.items():
            rec = load_record(path)
            # draft_records() says every draft was read from published
            # figures. That is wrong for a draft whose own source block says
            # otherwise (the tandem twist drafts are simulation), so the flag
            # is taken from the record.
            kind = (rec.get("source") or {}).get("kind")
            if kind and kind != "literature":
                q = {**q, "source": f"draft_from_{kind}"}
            items.append((path, rec, q, True))
    actuators, left_out = [], []
    for path, rec, quality, draft in items:
        rid = rec["id"]
        c = cav.get(rid) or {}
        if c.get("draw") is False:
            left_out.append({"id": rid, "reason": c.get("reason")})
            continue
        length = length_of(rec, estimates)
        L = length["value_m"]
        legs, top_rotvecs, why = legs_and_top_rotvecs(rec, L)
        if legs is None:
            left_out.append({"id": rid, "reason": why})
            continue
        physics, has_force, flags = _finger_facts(rec, quality)
        tasks = [task_of(rec, quality, legs, c, task_files[c["task"]])
                 for c in cones]
        # A record whose held-tip force is left off (data/map_force_withheld.json)
        # is treated as holding none: no force, no push, has_force false and
        # the no_force_data flag where Finger.flags() puts it. Its placement
        # on the task (task_of) still reads the record as stored.
        held_back = withheld.get(rid)
        if held_back is not None:
            has_force, force = False, None
            if "no_force_data" not in flags:
                at = (flags.index("no_motion_direction")
                      if "no_motion_direction" in flags else len(flags))
                flags = flags[:at] + ["no_force_data"] + flags[at:]
        else:
            force = force_of(rec, tasks)
        for cone, t in zip(cones, tasks):
            if pushes[cone["task"]] is not None and "reason" not in t:
                t["push"] = None if held_back is not None else push_of(
                    rec, quality, legs, force, t, cone,
                    task_files[cone["task"]],
                    pushes[cone["task"]]["needed_N"])
        surface = surface_of(rec, declared.get(rid), L, extras.get(rid))
        for cone, t in zip(cones, tasks):
            if cone["return_clear"] is not None and "reason" not in t:
                if cone["return_clear"].get("stroke_from") \
                        == STROKE_FROM_OWN_START:
                    t["return_clear"] = return_clear_own_start_of(
                        legs, surface, t, cone["return_clear"],
                        cone["stroke"]["travel_m"], cone["half_angle_deg"])
                    continue
                t["return_clear"] = return_clear_of(
                    legs, surface, t, cone["return_clear"],
                    cone["stroke"]["travel_m"])
        actuators.append({
            "id": rid, "name": rec.get("name") or rid, "physics": physics,
            "source_kind": (rec.get("source") or {}).get("kind"),
            "draft": draft, "hand_placed": False, "has_force": has_force,
            "flags": flags, "caveat": c.get("caveat"),
            "record_file": _rel(path), "record_sha256": sha256(path),
            "legs": legs, "surface": surface,
            "length": length,
            "bend_check": bend_check(legs, top_rotvecs, L),
            "tasks": tasks, "force": force,
            "force_withheld": (None if held_back is None
                               else held_back["reason"]),
            **(shown_as.get(rid) or NO_NAME),
            "thumbnail": pictures.get(rid)})
    if hand_placed:
        actuators += load_hand_placed(hand_placed, {a["id"] for a in actuators})
    for a in actuators:
        # a hand-placed actuator has no entry in either file
        for k, v in {**NO_NAME, "thumbnail": None}.items():
            a.setdefault(k, v)
    cone_docs = []
    for c in cones:
        about = drawn_direction(actuators, c["task"])
        for a in actuators:
            for t in a["tasks"] or []:
                if t["task"] == c["task"] and "needed_direction" in t:
                    t["off_drawn_direction_deg"] = angle_deg(
                        t["needed_direction"], about)
        push = pushes[c["task"]]
        if push is not None:
            with open(LOOKUP_OUT, encoding="utf-8") as fo:
                compared = set(json.load(fo)["separation"]["compared"])
            push["passers_hold_range_N"] = passers_range(actuators, c["task"],
                                                         compared)
        cone_docs.append({**c, "name": task_files[c["task"]].get("name"),
                          "axes": list(CONE_AXES), "drawn_direction": about,
                          "push": push})
    drawn_records = {rec["id"]: rec for _, rec, _, _ in items}
    scaled = {a["id"]: scaled_of(a, None if a["hand_placed"]
                                 else drawn_records[a["id"]],
                                 moduli, corrections)
              for a in actuators}
    return {"made_by": MADE_BY, "note": NOTE, "drafts_included": bool(drafts),
            "quantities": QUANTITIES, "length_tolerance": LENGTH_TOLERANCE,
            "task_cones": cone_docs, "force_note": FORCE_NOTE,
            "thumbnail_note": THUMBNAIL_NOTE,
            "actuators": actuators,
            "left_out": left_out,
            "force_withheld": withheld,
            "scaled": {"note": SCALED_NOTE,
                       "quantities": SCALED_QUANTITIES,
                       "force_quantities": SCALED_FORCE_QUANTITIES,
                       "pressure_scale": correction_docs,
                       "actuators": scaled}}


def _settled(x):
    """A number as it is written to disk: 12 significant figures, and nothing
    finer than 1e-12.

    numpy and scipy do not give the same last digits from one version or
    machine to the next. The lengths, angles and forces here are worth far
    fewer figures than that, so the digits that move are cut off, and a value
    that is only rounding noise about zero (1e-16 and below, where the
    smallest real values are about 1e-9) is written as 0.0. The file then
    comes out the same wherever it is built."""
    y = round(float(f"{x:.12g}"), 12)
    return 0.0 if y == 0 else y


def _settle(doc):
    """`doc` with every decimal number passed through `_settled`. Whole
    numbers, true/false and text are left as they are."""
    if isinstance(doc, float):
        return _settled(doc)
    if isinstance(doc, dict):
        return {k: _settle(v) for k, v in doc.items()}
    if isinstance(doc, (list, tuple)):
        return [_settle(v) for v in doc]
    return doc


def render(doc):
    """The document as the exact text written to disk."""
    return json.dumps(_settle(doc), indent=1, ensure_ascii=False,
                      allow_nan=False) + "\n"


def write(doc, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as fo:
        fo.write(render(doc))


def main(argv=None):
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--drafts", action="store_true")
    ap.add_argument("--site", action="store_true",
                    help="write site/data/map_paths.json (with drafts), "
                         "replacing it")
    ap.add_argument("--out", default=os.path.join("outputs", "map_paths"))
    ap.add_argument("--tag", default="")
    ap.add_argument("--task-cones", default=None,
                    help="another task cone file, in place of "
                         "data/map_task_cones.json; not with --site")
    a = ap.parse_args(argv)
    if a.site and a.task_cones:
        print("REFUSING: --site is written from data/map_task_cones.json only")
        return 1
    if a.site:
        path, drafts = SITE_FILE, True
    else:
        drafts = a.drafts
        path = os.path.join(ROOT, a.out, "map_paths"
                            + ("_with_drafts" if drafts else "")
                            + ("_" + a.tag if a.tag else "") + ".json")
        if os.path.exists(path):
            print("REFUSING to overwrite:", path)
            return 1
    doc = (build(drafts) if a.task_cones is None
           else build(drafts, task_cones=a.task_cones))
    write(doc, path)
    print("wrote", path)
    print(f"{len(doc['actuators'])} actuators, "
          f"{sum(len(x['legs']) for x in doc['actuators'])} legs")
    for lo in doc["left_out"]:
        print("left out:", lo["id"], "-", lo["reason"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
