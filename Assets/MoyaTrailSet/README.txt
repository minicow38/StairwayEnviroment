Moya Trail Set (URP)

Files:
- MoyaTrailTexture.png
- MoyaTrailURP.shader
- MoyaTrail.mat

Usage:
1. Copy the entire MoyaTrailSet folder into your Unity project's Assets folder.
2. Wait for Unity to import the files.
3. Drag MoyaTrail.mat into Trail Renderer > Materials > Element 0.

Recommended Trail Renderer values:
- Time: 0.15 - 0.25
- Width: about 0.06 - 0.10
- Min Vertex Distance: 0.12 - 0.18
- Texture Mode: Stretch
- Alignment: View if the source object rotates strongly
- Color gradient: head alpha ~0.6-0.8, tail alpha 0

Material tuning:
- Tint: pale cyan / white-blue
- Glow: 0.4 - 1.2
- Opacity: 0.5 - 0.8
- Soft Edge: 0.9 - 1.4

Notes:
- This shader is for URP.
- If the result looks too much like a laser, lower Glow and Opacity.
- If the mist is too faint, raise Opacity first, then Glow.
