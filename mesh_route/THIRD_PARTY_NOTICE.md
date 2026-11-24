# Third-party notice for two files in this folder

`parametricGripper.py` and `parametricGripperController.py` were written
starting from the PneunetGripper tutorial of the SoftRobots plugin for SOFA
(`tutorials/PneunetGripper/` in https://github.com/SofaDefrost/SoftRobots;
the copy we worked from was the plugin shipped with SOFA v24.06, commit
`9f18097c5d3da30a8e64f5db6ae1eeaa251cf7eb`).

- Author of the tutorial: team DEFROST (INRIA, University of Lille, CNRS,
  Ecole Centrale de Lille).
- Its licence: the GNU Lesser General Public License. The plugin's
  `LICENSE` file is version 3, copied here unchanged as
  `LICENSE.SoftRobots-LGPL-3.0.txt`. The plugin's source header says
  version 2.1 or, at your option, any later version. Version 3 adds
  permissions to the GNU General Public License version 3, whose text is at
  https://www.gnu.org/licenses/gpl-3.0.txt.

What comes from the tutorial: the layout of the scene (the order of solvers,
the finger node with its cavity sub-node and pressure constraint), the idea
of a controller that raises and lowers the pressure with the + and - keys,
and a branch of `parametricGripper.py` (`USE_CUSTOM_CAD = False`) that loads
the tutorial's own meshes with the tutorial's numbers. Those meshes are not
in this repository. The rest of the two files is our own.

We make these two files available under the same GNU Lesser General Public
License, version 3 or later. Every other file in this repository stays under
the repository's own licence (Apache-2.0, see `LICENSE` at the top).

The tutorial's files carry no licence header of their own, and neither do
ours. This note is our reading of where the two files come from; it is not
legal advice.
