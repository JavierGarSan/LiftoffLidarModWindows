using BepInEx;
using BepInEx.Logging;
using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Linq.Expressions;


namespace LiftoffLidarMod
{
    [BepInPlugin("com.jacob.lidarmod", "Liftoff Lidar", "1.0.0")]
    public class Plugin : BaseUnityPlugin
    {
        internal static ManualLogSource Log;
        private UdpClient udpClient;
        private IPEndPoint remoteEndPoint;
        int updateCounter = 0;
        private const int N_POINTS = 180;

        // physics_ticks_per_lidar_packet, a value of 0 ensures every physics tick the LiDARs are cast (50Hz). If a value of 3 is chosen LiDARs are cast every 3 physics ticks.
        private const int physics_ticks_per_lidar_packet = 1;

        // Use jacob rings uses 360 points. Distirubted across 2 rings, one in uv and one in vw.
        // Else it uses a 21x15 grid and 45 misc points. These have been prebaked for speed reasons/lazyness. Sorry.
        private const bool use_jacob_rings = false;

        // Codex code begins here: This should be a list of all the prebaked points
        // Local-space unit directions: +X right, +Y up, +Z forward.
        // Packet float indices match this array. Angles below are yaw/elevation in degrees.
        // Grid: index = row * 21 + column; yaw = -60 + column * 6;
        // elevation = (row - 7) * 45 / 7. Straight ahead is index 157.
        private static readonly Vector3[] PrebakedRayDirections = new Vector3[]
        {
            // Forward grid row 0: indices 0-20, elevation -45.
            new Vector3(-0.612372436f, -0.707106781f, 0.353553391f), // 0: yaw -60, elevation -45
            new Vector3(-0.572061403f, -0.707106781f, 0.415626938f), // 1: yaw -54, elevation -45
            new Vector3(-0.525482745f, -0.707106781f, 0.473146789f), // 2: yaw -48, elevation -45
            new Vector3(-0.473146789f, -0.707106781f, 0.525482745f), // 3: yaw -42, elevation -45
            new Vector3(-0.415626938f, -0.707106781f, 0.572061403f), // 4: yaw -36, elevation -45
            new Vector3(-0.353553391f, -0.707106781f, 0.612372436f), // 5: yaw -30, elevation -45
            new Vector3(-0.287606238f, -0.707106781f, 0.645974188f), // 6: yaw -24, elevation -45
            new Vector3(-0.218508012f, -0.707106781f, 0.672498512f), // 7: yaw -18, elevation -45
            new Vector3(-0.147015766f, -0.707106781f, 0.691654801f), // 8: yaw -12, elevation -45
            new Vector3(-0.073912785f, -0.707106781f, 0.703233176f), // 9: yaw -6, elevation -45
            new Vector3(0.000000000f, -0.707106781f, 0.707106781f), // 10: yaw 0, elevation -45
            new Vector3(0.073912785f, -0.707106781f, 0.703233176f), // 11: yaw 6, elevation -45
            new Vector3(0.147015766f, -0.707106781f, 0.691654801f), // 12: yaw 12, elevation -45
            new Vector3(0.218508012f, -0.707106781f, 0.672498512f), // 13: yaw 18, elevation -45
            new Vector3(0.287606238f, -0.707106781f, 0.645974188f), // 14: yaw 24, elevation -45
            new Vector3(0.353553391f, -0.707106781f, 0.612372436f), // 15: yaw 30, elevation -45
            new Vector3(0.415626938f, -0.707106781f, 0.572061403f), // 16: yaw 36, elevation -45
            new Vector3(0.473146789f, -0.707106781f, 0.525482745f), // 17: yaw 42, elevation -45
            new Vector3(0.525482745f, -0.707106781f, 0.473146789f), // 18: yaw 48, elevation -45
            new Vector3(0.572061403f, -0.707106781f, 0.415626938f), // 19: yaw 54, elevation -45
            new Vector3(0.612372436f, -0.707106781f, 0.353553391f), // 20: yaw 60, elevation -45
            // Forward grid row 1: indices 21-41, elevation -38.571429.
            new Vector3(-0.677085925f, -0.623489802f, 0.390915741f), // 21: yaw -60, elevation -38.571429
            new Vector3(-0.632514956f, -0.623489802f, 0.459549015f), // 22: yaw -54, elevation -38.571429
            new Vector3(-0.581014021f, -0.623489802f, 0.523147374f), // 23: yaw -48, elevation -38.571429
            new Vector3(-0.523147374f, -0.623489802f, 0.581014021f), // 24: yaw -42, elevation -38.571429
            new Vector3(-0.459549015f, -0.623489802f, 0.632514956f), // 25: yaw -36, elevation -38.571429
            new Vector3(-0.390915741f, -0.623489802f, 0.677085925f), // 26: yaw -30, elevation -38.571429
            new Vector3(-0.317999513f, -0.623489802f, 0.714238599f), // 27: yaw -24, elevation -38.571429
            new Vector3(-0.241599215f, -0.623489802f, 0.743565926f), // 28: yaw -18, elevation -38.571429
            new Vector3(-0.162551905f, -0.623489802f, 0.764746589f), // 29: yaw -12, elevation -38.571429
            new Vector3(-0.081723643f, -0.623489802f, 0.777548528f), // 30: yaw -6, elevation -38.571429
            new Vector3(0.000000000f, -0.623489802f, 0.781831482f), // 31: yaw 0, elevation -38.571429
            new Vector3(0.081723643f, -0.623489802f, 0.777548528f), // 32: yaw 6, elevation -38.571429
            new Vector3(0.162551905f, -0.623489802f, 0.764746589f), // 33: yaw 12, elevation -38.571429
            new Vector3(0.241599215f, -0.623489802f, 0.743565926f), // 34: yaw 18, elevation -38.571429
            new Vector3(0.317999513f, -0.623489802f, 0.714238599f), // 35: yaw 24, elevation -38.571429
            new Vector3(0.390915741f, -0.623489802f, 0.677085925f), // 36: yaw 30, elevation -38.571429
            new Vector3(0.459549015f, -0.623489802f, 0.632514956f), // 37: yaw 36, elevation -38.571429
            new Vector3(0.523147374f, -0.623489802f, 0.581014021f), // 38: yaw 42, elevation -38.571429
            new Vector3(0.581014021f, -0.623489802f, 0.523147374f), // 39: yaw 48, elevation -38.571429
            new Vector3(0.632514956f, -0.623489802f, 0.459549015f), // 40: yaw 54, elevation -38.571429
            new Vector3(0.677085925f, -0.623489802f, 0.390915741f), // 41: yaw 60, elevation -38.571429
            // Forward grid row 2: indices 42-62, elevation -32.142857.
            new Vector3(-0.733284667f, -0.532032077f, 0.423362100f), // 42: yaw -60, elevation -32.142857
            new Vector3(-0.685014267f, -0.532032077f, 0.497691997f), // 43: yaw -54, elevation -32.142857
            new Vector3(-0.629238707f, -0.532032077f, 0.566569077f), // 44: yaw -48, elevation -32.142857
            new Vector3(-0.566569077f, -0.532032077f, 0.629238707f), // 45: yaw -42, elevation -32.142857
            new Vector3(-0.497691997f, -0.532032077f, 0.685014267f), // 46: yaw -36, elevation -32.142857
            new Vector3(-0.423362100f, -0.532032077f, 0.733284667f), // 47: yaw -30, elevation -32.142857
            new Vector3(-0.344393758f, -0.532032077f, 0.773521046f), // 48: yaw -24, elevation -32.142857
            new Vector3(-0.261652167f, -0.532032077f, 0.805282567f), // 49: yaw -18, elevation -32.142857
            new Vector3(-0.176043860f, -0.532032077f, 0.828221244f), // 50: yaw -12, elevation -32.142857
            new Vector3(-0.088506779f, -0.532032077f, 0.842085755f), // 51: yaw -6, elevation -32.142857
            new Vector3(0.000000000f, -0.532032077f, 0.846724199f), // 52: yaw 0, elevation -32.142857
            new Vector3(0.088506779f, -0.532032077f, 0.842085755f), // 53: yaw 6, elevation -32.142857
            new Vector3(0.176043860f, -0.532032077f, 0.828221244f), // 54: yaw 12, elevation -32.142857
            new Vector3(0.261652167f, -0.532032077f, 0.805282567f), // 55: yaw 18, elevation -32.142857
            new Vector3(0.344393758f, -0.532032077f, 0.773521046f), // 56: yaw 24, elevation -32.142857
            new Vector3(0.423362100f, -0.532032077f, 0.733284667f), // 57: yaw 30, elevation -32.142857
            new Vector3(0.497691997f, -0.532032077f, 0.685014267f), // 58: yaw 36, elevation -32.142857
            new Vector3(0.566569077f, -0.532032077f, 0.629238707f), // 59: yaw 42, elevation -32.142857
            new Vector3(0.629238707f, -0.532032077f, 0.566569077f), // 60: yaw 48, elevation -32.142857
            new Vector3(0.685014267f, -0.532032077f, 0.497691997f), // 61: yaw 54, elevation -32.142857
            new Vector3(0.733284667f, -0.532032077f, 0.423362100f), // 62: yaw 60, elevation -32.142857
            // Forward grid row 3: indices 63-83, elevation -25.714286.
            new Vector3(-0.780261928f, -0.433883739f, 0.450484434f), // 63: yaw -60, elevation -25.714286
            new Vector3(-0.728899126f, -0.433883739f, 0.529576213f), // 64: yaw -54, elevation -25.714286
            new Vector3(-0.669550352f, -0.433883739f, 0.602865845f), // 65: yaw -48, elevation -25.714286
            new Vector3(-0.602865845f, -0.433883739f, 0.669550352f), // 66: yaw -42, elevation -25.714286
            new Vector3(-0.529576213f, -0.433883739f, 0.728899126f), // 67: yaw -36, elevation -25.714286
            new Vector3(-0.450484434f, -0.433883739f, 0.780261928f), // 68: yaw -30, elevation -25.714286
            new Vector3(-0.366457053f, -0.433883739f, 0.823076017f), // 69: yaw -24, elevation -25.714286
            new Vector3(-0.278414692f, -0.433883739f, 0.856872313f), // 70: yaw -18, elevation -25.714286
            new Vector3(-0.187321961f, -0.433883739f, 0.881280536f), // 71: yaw -12, elevation -25.714286
            new Vector3(-0.094176891f, -0.433883739f, 0.896033266f), // 72: yaw -6, elevation -25.714286
            new Vector3(0.000000000f, -0.433883739f, 0.900968868f), // 73: yaw 0, elevation -25.714286
            new Vector3(0.094176891f, -0.433883739f, 0.896033266f), // 74: yaw 6, elevation -25.714286
            new Vector3(0.187321961f, -0.433883739f, 0.881280536f), // 75: yaw 12, elevation -25.714286
            new Vector3(0.278414692f, -0.433883739f, 0.856872313f), // 76: yaw 18, elevation -25.714286
            new Vector3(0.366457053f, -0.433883739f, 0.823076017f), // 77: yaw 24, elevation -25.714286
            new Vector3(0.450484434f, -0.433883739f, 0.780261928f), // 78: yaw 30, elevation -25.714286
            new Vector3(0.529576213f, -0.433883739f, 0.728899126f), // 79: yaw 36, elevation -25.714286
            new Vector3(0.602865845f, -0.433883739f, 0.669550352f), // 80: yaw 42, elevation -25.714286
            new Vector3(0.669550352f, -0.433883739f, 0.602865845f), // 81: yaw 48, elevation -25.714286
            new Vector3(0.728899126f, -0.433883739f, 0.529576213f), // 82: yaw 54, elevation -25.714286
            new Vector3(0.780261928f, -0.433883739f, 0.450484434f), // 83: yaw 60, elevation -25.714286
            // Forward grid row 4: indices 84-104, elevation -19.285714.
            new Vector3(-0.817426942f, -0.330279062f, 0.471941665f), // 84: yaw -60, elevation -19.285714
            new Vector3(-0.763617655f, -0.330279062f, 0.554800701f), // 85: yaw -54, elevation -19.285714
            new Vector3(-0.701442013f, -0.330279062f, 0.631581225f), // 86: yaw -48, elevation -19.285714
            new Vector3(-0.631581225f, -0.330279062f, 0.701442013f), // 87: yaw -42, elevation -19.285714
            new Vector3(-0.554800701f, -0.330279062f, 0.763617655f), // 88: yaw -36, elevation -19.285714
            new Vector3(-0.471941665f, -0.330279062f, 0.817426942f), // 89: yaw -30, elevation -19.285714
            new Vector3(-0.383911937f, -0.330279062f, 0.862280329f), // 90: yaw -24, elevation -19.285714
            new Vector3(-0.291675990f, -0.330279062f, 0.897686392f), // 91: yaw -18, elevation -19.285714
            new Vector3(-0.196244379f, -0.330279062f, 0.923257215f), // 92: yaw -12, elevation -19.285714
            new Vector3(-0.098662674f, -0.330279062f, 0.938712639f), // 93: yaw -6, elevation -19.285714
            new Vector3(0.000000000f, -0.330279062f, 0.943883330f), // 94: yaw 0, elevation -19.285714
            new Vector3(0.098662674f, -0.330279062f, 0.938712639f), // 95: yaw 6, elevation -19.285714
            new Vector3(0.196244379f, -0.330279062f, 0.923257215f), // 96: yaw 12, elevation -19.285714
            new Vector3(0.291675990f, -0.330279062f, 0.897686392f), // 97: yaw 18, elevation -19.285714
            new Vector3(0.383911937f, -0.330279062f, 0.862280329f), // 98: yaw 24, elevation -19.285714
            new Vector3(0.471941665f, -0.330279062f, 0.817426942f), // 99: yaw 30, elevation -19.285714
            new Vector3(0.554800701f, -0.330279062f, 0.763617655f), // 100: yaw 36, elevation -19.285714
            new Vector3(0.631581225f, -0.330279062f, 0.701442013f), // 101: yaw 42, elevation -19.285714
            new Vector3(0.701442013f, -0.330279062f, 0.631581225f), // 102: yaw 48, elevation -19.285714
            new Vector3(0.763617655f, -0.330279062f, 0.554800701f), // 103: yaw 54, elevation -19.285714
            new Vector3(0.817426942f, -0.330279062f, 0.471941665f), // 104: yaw 60, elevation -19.285714
            // Forward grid row 5: indices 105-125, elevation -12.857143.
            new Vector3(-0.844312339f, -0.222520934f, 0.487463956f), // 105: yaw -60, elevation -12.857143
            new Vector3(-0.788733249f, -0.222520934f, 0.573048249f), // 106: yaw -54, elevation -12.857143
            new Vector3(-0.724512633f, -0.222520934f, 0.652354105f), // 107: yaw -48, elevation -12.857143
            new Vector3(-0.652354105f, -0.222520934f, 0.724512633f), // 108: yaw -42, elevation -12.857143
            new Vector3(-0.573048249f, -0.222520934f, 0.788733249f), // 109: yaw -36, elevation -12.857143
            new Vector3(-0.487463956f, -0.222520934f, 0.844312339f), // 110: yaw -30, elevation -12.857143
            new Vector3(-0.396538906f, -0.222520934f, 0.890640966f), // 111: yaw -24, elevation -12.857143
            new Vector3(-0.301269293f, -0.222520934f, 0.927211544f), // 112: yaw -18, elevation -12.857143
            new Vector3(-0.202698911f, -0.222520934f, 0.953623398f), // 113: yaw -12, elevation -12.857143
            new Vector3(-0.101907716f, -0.222520934f, 0.969587155f), // 114: yaw -6, elevation -12.857143
            new Vector3(0.000000000f, -0.222520934f, 0.974927912f), // 115: yaw 0, elevation -12.857143
            new Vector3(0.101907716f, -0.222520934f, 0.969587155f), // 116: yaw 6, elevation -12.857143
            new Vector3(0.202698911f, -0.222520934f, 0.953623398f), // 117: yaw 12, elevation -12.857143
            new Vector3(0.301269293f, -0.222520934f, 0.927211544f), // 118: yaw 18, elevation -12.857143
            new Vector3(0.396538906f, -0.222520934f, 0.890640966f), // 119: yaw 24, elevation -12.857143
            new Vector3(0.487463956f, -0.222520934f, 0.844312339f), // 120: yaw 30, elevation -12.857143
            new Vector3(0.573048249f, -0.222520934f, 0.788733249f), // 121: yaw 36, elevation -12.857143
            new Vector3(0.652354105f, -0.222520934f, 0.724512633f), // 122: yaw 42, elevation -12.857143
            new Vector3(0.724512633f, -0.222520934f, 0.652354105f), // 123: yaw 48, elevation -12.857143
            new Vector3(0.788733249f, -0.222520934f, 0.573048249f), // 124: yaw 54, elevation -12.857143
            new Vector3(0.844312339f, -0.222520934f, 0.487463956f), // 125: yaw 60, elevation -12.857143
            // Forward grid row 6: indices 126-146, elevation -6.428571.
            new Vector3(-0.860580018f, -0.111964476f, 0.496856105f), // 126: yaw -60, elevation -6.428571
            new Vector3(-0.803930065f, -0.111964476f, 0.584089382f), // 127: yaw -54, elevation -6.428571
            new Vector3(-0.738472087f, -0.111964476f, 0.664923254f), // 128: yaw -48, elevation -6.428571
            new Vector3(-0.664923254f, -0.111964476f, 0.738472087f), // 129: yaw -42, elevation -6.428571
            new Vector3(-0.584089382f, -0.111964476f, 0.803930065f), // 130: yaw -36, elevation -6.428571
            new Vector3(-0.496856105f, -0.111964476f, 0.860580018f), // 131: yaw -30, elevation -6.428571
            new Vector3(-0.404179168f, -0.111964476f, 0.907801276f), // 132: yaw -24, elevation -6.428571
            new Vector3(-0.307073960f, -0.111964476f, 0.945076473f), // 133: yaw -18, elevation -6.428571
            new Vector3(-0.206604386f, -0.111964476f, 0.971997214f), // 134: yaw -12, elevation -6.428571
            new Vector3(-0.103871210f, -0.111964476f, 0.988268550f), // 135: yaw -6, elevation -6.428571
            new Vector3(0.000000000f, -0.111964476f, 0.993712210f), // 136: yaw 0, elevation -6.428571
            new Vector3(0.103871210f, -0.111964476f, 0.988268550f), // 137: yaw 6, elevation -6.428571
            new Vector3(0.206604386f, -0.111964476f, 0.971997214f), // 138: yaw 12, elevation -6.428571
            new Vector3(0.307073960f, -0.111964476f, 0.945076473f), // 139: yaw 18, elevation -6.428571
            new Vector3(0.404179168f, -0.111964476f, 0.907801276f), // 140: yaw 24, elevation -6.428571
            new Vector3(0.496856105f, -0.111964476f, 0.860580018f), // 141: yaw 30, elevation -6.428571
            new Vector3(0.584089382f, -0.111964476f, 0.803930065f), // 142: yaw 36, elevation -6.428571
            new Vector3(0.664923254f, -0.111964476f, 0.738472087f), // 143: yaw 42, elevation -6.428571
            new Vector3(0.738472087f, -0.111964476f, 0.664923254f), // 144: yaw 48, elevation -6.428571
            new Vector3(0.803930065f, -0.111964476f, 0.584089382f), // 145: yaw 54, elevation -6.428571
            new Vector3(0.860580018f, -0.111964476f, 0.496856105f), // 146: yaw 60, elevation -6.428571
            // Forward grid row 7: indices 147-167, elevation 0.
            new Vector3(-0.866025404f, 0.000000000f, 0.500000000f), // 147: yaw -60, elevation 0
            new Vector3(-0.809016994f, 0.000000000f, 0.587785252f), // 148: yaw -54, elevation 0
            new Vector3(-0.743144825f, 0.000000000f, 0.669130606f), // 149: yaw -48, elevation 0
            new Vector3(-0.669130606f, 0.000000000f, 0.743144825f), // 150: yaw -42, elevation 0
            new Vector3(-0.587785252f, 0.000000000f, 0.809016994f), // 151: yaw -36, elevation 0
            new Vector3(-0.500000000f, 0.000000000f, 0.866025404f), // 152: yaw -30, elevation 0
            new Vector3(-0.406736643f, 0.000000000f, 0.913545458f), // 153: yaw -24, elevation 0
            new Vector3(-0.309016994f, 0.000000000f, 0.951056516f), // 154: yaw -18, elevation 0
            new Vector3(-0.207911691f, 0.000000000f, 0.978147601f), // 155: yaw -12, elevation 0
            new Vector3(-0.104528463f, 0.000000000f, 0.994521895f), // 156: yaw -6, elevation 0
            new Vector3(0.000000000f, 0.000000000f, 1.000000000f), // 157: yaw 0, elevation 0
            new Vector3(0.104528463f, 0.000000000f, 0.994521895f), // 158: yaw 6, elevation 0
            new Vector3(0.207911691f, 0.000000000f, 0.978147601f), // 159: yaw 12, elevation 0
            new Vector3(0.309016994f, 0.000000000f, 0.951056516f), // 160: yaw 18, elevation 0
            new Vector3(0.406736643f, 0.000000000f, 0.913545458f), // 161: yaw 24, elevation 0
            new Vector3(0.500000000f, 0.000000000f, 0.866025404f), // 162: yaw 30, elevation 0
            new Vector3(0.587785252f, 0.000000000f, 0.809016994f), // 163: yaw 36, elevation 0
            new Vector3(0.669130606f, 0.000000000f, 0.743144825f), // 164: yaw 42, elevation 0
            new Vector3(0.743144825f, 0.000000000f, 0.669130606f), // 165: yaw 48, elevation 0
            new Vector3(0.809016994f, 0.000000000f, 0.587785252f), // 166: yaw 54, elevation 0
            new Vector3(0.866025404f, 0.000000000f, 0.500000000f), // 167: yaw 60, elevation 0
            // Forward grid row 8: indices 168-188, elevation 6.428571.
            new Vector3(-0.860580018f, 0.111964476f, 0.496856105f), // 168: yaw -60, elevation 6.428571
            new Vector3(-0.803930065f, 0.111964476f, 0.584089382f), // 169: yaw -54, elevation 6.428571
            new Vector3(-0.738472087f, 0.111964476f, 0.664923254f), // 170: yaw -48, elevation 6.428571
            new Vector3(-0.664923254f, 0.111964476f, 0.738472087f), // 171: yaw -42, elevation 6.428571
            new Vector3(-0.584089382f, 0.111964476f, 0.803930065f), // 172: yaw -36, elevation 6.428571
            new Vector3(-0.496856105f, 0.111964476f, 0.860580018f), // 173: yaw -30, elevation 6.428571
            new Vector3(-0.404179168f, 0.111964476f, 0.907801276f), // 174: yaw -24, elevation 6.428571
            new Vector3(-0.307073960f, 0.111964476f, 0.945076473f), // 175: yaw -18, elevation 6.428571
            new Vector3(-0.206604386f, 0.111964476f, 0.971997214f), // 176: yaw -12, elevation 6.428571
            new Vector3(-0.103871210f, 0.111964476f, 0.988268550f), // 177: yaw -6, elevation 6.428571
            new Vector3(0.000000000f, 0.111964476f, 0.993712210f), // 178: yaw 0, elevation 6.428571
            new Vector3(0.103871210f, 0.111964476f, 0.988268550f), // 179: yaw 6, elevation 6.428571
            new Vector3(0.206604386f, 0.111964476f, 0.971997214f), // 180: yaw 12, elevation 6.428571
            new Vector3(0.307073960f, 0.111964476f, 0.945076473f), // 181: yaw 18, elevation 6.428571
            new Vector3(0.404179168f, 0.111964476f, 0.907801276f), // 182: yaw 24, elevation 6.428571
            new Vector3(0.496856105f, 0.111964476f, 0.860580018f), // 183: yaw 30, elevation 6.428571
            new Vector3(0.584089382f, 0.111964476f, 0.803930065f), // 184: yaw 36, elevation 6.428571
            new Vector3(0.664923254f, 0.111964476f, 0.738472087f), // 185: yaw 42, elevation 6.428571
            new Vector3(0.738472087f, 0.111964476f, 0.664923254f), // 186: yaw 48, elevation 6.428571
            new Vector3(0.803930065f, 0.111964476f, 0.584089382f), // 187: yaw 54, elevation 6.428571
            new Vector3(0.860580018f, 0.111964476f, 0.496856105f), // 188: yaw 60, elevation 6.428571
            // Forward grid row 9: indices 189-209, elevation 12.857143.
            new Vector3(-0.844312339f, 0.222520934f, 0.487463956f), // 189: yaw -60, elevation 12.857143
            new Vector3(-0.788733249f, 0.222520934f, 0.573048249f), // 190: yaw -54, elevation 12.857143
            new Vector3(-0.724512633f, 0.222520934f, 0.652354105f), // 191: yaw -48, elevation 12.857143
            new Vector3(-0.652354105f, 0.222520934f, 0.724512633f), // 192: yaw -42, elevation 12.857143
            new Vector3(-0.573048249f, 0.222520934f, 0.788733249f), // 193: yaw -36, elevation 12.857143
            new Vector3(-0.487463956f, 0.222520934f, 0.844312339f), // 194: yaw -30, elevation 12.857143
            new Vector3(-0.396538906f, 0.222520934f, 0.890640966f), // 195: yaw -24, elevation 12.857143
            new Vector3(-0.301269293f, 0.222520934f, 0.927211544f), // 196: yaw -18, elevation 12.857143
            new Vector3(-0.202698911f, 0.222520934f, 0.953623398f), // 197: yaw -12, elevation 12.857143
            new Vector3(-0.101907716f, 0.222520934f, 0.969587155f), // 198: yaw -6, elevation 12.857143
            new Vector3(0.000000000f, 0.222520934f, 0.974927912f), // 199: yaw 0, elevation 12.857143
            new Vector3(0.101907716f, 0.222520934f, 0.969587155f), // 200: yaw 6, elevation 12.857143
            new Vector3(0.202698911f, 0.222520934f, 0.953623398f), // 201: yaw 12, elevation 12.857143
            new Vector3(0.301269293f, 0.222520934f, 0.927211544f), // 202: yaw 18, elevation 12.857143
            new Vector3(0.396538906f, 0.222520934f, 0.890640966f), // 203: yaw 24, elevation 12.857143
            new Vector3(0.487463956f, 0.222520934f, 0.844312339f), // 204: yaw 30, elevation 12.857143
            new Vector3(0.573048249f, 0.222520934f, 0.788733249f), // 205: yaw 36, elevation 12.857143
            new Vector3(0.652354105f, 0.222520934f, 0.724512633f), // 206: yaw 42, elevation 12.857143
            new Vector3(0.724512633f, 0.222520934f, 0.652354105f), // 207: yaw 48, elevation 12.857143
            new Vector3(0.788733249f, 0.222520934f, 0.573048249f), // 208: yaw 54, elevation 12.857143
            new Vector3(0.844312339f, 0.222520934f, 0.487463956f), // 209: yaw 60, elevation 12.857143
            // Forward grid row 10: indices 210-230, elevation 19.285714.
            new Vector3(-0.817426942f, 0.330279062f, 0.471941665f), // 210: yaw -60, elevation 19.285714
            new Vector3(-0.763617655f, 0.330279062f, 0.554800701f), // 211: yaw -54, elevation 19.285714
            new Vector3(-0.701442013f, 0.330279062f, 0.631581225f), // 212: yaw -48, elevation 19.285714
            new Vector3(-0.631581225f, 0.330279062f, 0.701442013f), // 213: yaw -42, elevation 19.285714
            new Vector3(-0.554800701f, 0.330279062f, 0.763617655f), // 214: yaw -36, elevation 19.285714
            new Vector3(-0.471941665f, 0.330279062f, 0.817426942f), // 215: yaw -30, elevation 19.285714
            new Vector3(-0.383911937f, 0.330279062f, 0.862280329f), // 216: yaw -24, elevation 19.285714
            new Vector3(-0.291675990f, 0.330279062f, 0.897686392f), // 217: yaw -18, elevation 19.285714
            new Vector3(-0.196244379f, 0.330279062f, 0.923257215f), // 218: yaw -12, elevation 19.285714
            new Vector3(-0.098662674f, 0.330279062f, 0.938712639f), // 219: yaw -6, elevation 19.285714
            new Vector3(0.000000000f, 0.330279062f, 0.943883330f), // 220: yaw 0, elevation 19.285714
            new Vector3(0.098662674f, 0.330279062f, 0.938712639f), // 221: yaw 6, elevation 19.285714
            new Vector3(0.196244379f, 0.330279062f, 0.923257215f), // 222: yaw 12, elevation 19.285714
            new Vector3(0.291675990f, 0.330279062f, 0.897686392f), // 223: yaw 18, elevation 19.285714
            new Vector3(0.383911937f, 0.330279062f, 0.862280329f), // 224: yaw 24, elevation 19.285714
            new Vector3(0.471941665f, 0.330279062f, 0.817426942f), // 225: yaw 30, elevation 19.285714
            new Vector3(0.554800701f, 0.330279062f, 0.763617655f), // 226: yaw 36, elevation 19.285714
            new Vector3(0.631581225f, 0.330279062f, 0.701442013f), // 227: yaw 42, elevation 19.285714
            new Vector3(0.701442013f, 0.330279062f, 0.631581225f), // 228: yaw 48, elevation 19.285714
            new Vector3(0.763617655f, 0.330279062f, 0.554800701f), // 229: yaw 54, elevation 19.285714
            new Vector3(0.817426942f, 0.330279062f, 0.471941665f), // 230: yaw 60, elevation 19.285714
            // Forward grid row 11: indices 231-251, elevation 25.714286.
            new Vector3(-0.780261928f, 0.433883739f, 0.450484434f), // 231: yaw -60, elevation 25.714286
            new Vector3(-0.728899126f, 0.433883739f, 0.529576213f), // 232: yaw -54, elevation 25.714286
            new Vector3(-0.669550352f, 0.433883739f, 0.602865845f), // 233: yaw -48, elevation 25.714286
            new Vector3(-0.602865845f, 0.433883739f, 0.669550352f), // 234: yaw -42, elevation 25.714286
            new Vector3(-0.529576213f, 0.433883739f, 0.728899126f), // 235: yaw -36, elevation 25.714286
            new Vector3(-0.450484434f, 0.433883739f, 0.780261928f), // 236: yaw -30, elevation 25.714286
            new Vector3(-0.366457053f, 0.433883739f, 0.823076017f), // 237: yaw -24, elevation 25.714286
            new Vector3(-0.278414692f, 0.433883739f, 0.856872313f), // 238: yaw -18, elevation 25.714286
            new Vector3(-0.187321961f, 0.433883739f, 0.881280536f), // 239: yaw -12, elevation 25.714286
            new Vector3(-0.094176891f, 0.433883739f, 0.896033266f), // 240: yaw -6, elevation 25.714286
            new Vector3(0.000000000f, 0.433883739f, 0.900968868f), // 241: yaw 0, elevation 25.714286
            new Vector3(0.094176891f, 0.433883739f, 0.896033266f), // 242: yaw 6, elevation 25.714286
            new Vector3(0.187321961f, 0.433883739f, 0.881280536f), // 243: yaw 12, elevation 25.714286
            new Vector3(0.278414692f, 0.433883739f, 0.856872313f), // 244: yaw 18, elevation 25.714286
            new Vector3(0.366457053f, 0.433883739f, 0.823076017f), // 245: yaw 24, elevation 25.714286
            new Vector3(0.450484434f, 0.433883739f, 0.780261928f), // 246: yaw 30, elevation 25.714286
            new Vector3(0.529576213f, 0.433883739f, 0.728899126f), // 247: yaw 36, elevation 25.714286
            new Vector3(0.602865845f, 0.433883739f, 0.669550352f), // 248: yaw 42, elevation 25.714286
            new Vector3(0.669550352f, 0.433883739f, 0.602865845f), // 249: yaw 48, elevation 25.714286
            new Vector3(0.728899126f, 0.433883739f, 0.529576213f), // 250: yaw 54, elevation 25.714286
            new Vector3(0.780261928f, 0.433883739f, 0.450484434f), // 251: yaw 60, elevation 25.714286
            // Forward grid row 12: indices 252-272, elevation 32.142857.
            new Vector3(-0.733284667f, 0.532032077f, 0.423362100f), // 252: yaw -60, elevation 32.142857
            new Vector3(-0.685014267f, 0.532032077f, 0.497691997f), // 253: yaw -54, elevation 32.142857
            new Vector3(-0.629238707f, 0.532032077f, 0.566569077f), // 254: yaw -48, elevation 32.142857
            new Vector3(-0.566569077f, 0.532032077f, 0.629238707f), // 255: yaw -42, elevation 32.142857
            new Vector3(-0.497691997f, 0.532032077f, 0.685014267f), // 256: yaw -36, elevation 32.142857
            new Vector3(-0.423362100f, 0.532032077f, 0.733284667f), // 257: yaw -30, elevation 32.142857
            new Vector3(-0.344393758f, 0.532032077f, 0.773521046f), // 258: yaw -24, elevation 32.142857
            new Vector3(-0.261652167f, 0.532032077f, 0.805282567f), // 259: yaw -18, elevation 32.142857
            new Vector3(-0.176043860f, 0.532032077f, 0.828221244f), // 260: yaw -12, elevation 32.142857
            new Vector3(-0.088506779f, 0.532032077f, 0.842085755f), // 261: yaw -6, elevation 32.142857
            new Vector3(0.000000000f, 0.532032077f, 0.846724199f), // 262: yaw 0, elevation 32.142857
            new Vector3(0.088506779f, 0.532032077f, 0.842085755f), // 263: yaw 6, elevation 32.142857
            new Vector3(0.176043860f, 0.532032077f, 0.828221244f), // 264: yaw 12, elevation 32.142857
            new Vector3(0.261652167f, 0.532032077f, 0.805282567f), // 265: yaw 18, elevation 32.142857
            new Vector3(0.344393758f, 0.532032077f, 0.773521046f), // 266: yaw 24, elevation 32.142857
            new Vector3(0.423362100f, 0.532032077f, 0.733284667f), // 267: yaw 30, elevation 32.142857
            new Vector3(0.497691997f, 0.532032077f, 0.685014267f), // 268: yaw 36, elevation 32.142857
            new Vector3(0.566569077f, 0.532032077f, 0.629238707f), // 269: yaw 42, elevation 32.142857
            new Vector3(0.629238707f, 0.532032077f, 0.566569077f), // 270: yaw 48, elevation 32.142857
            new Vector3(0.685014267f, 0.532032077f, 0.497691997f), // 271: yaw 54, elevation 32.142857
            new Vector3(0.733284667f, 0.532032077f, 0.423362100f), // 272: yaw 60, elevation 32.142857
            // Forward grid row 13: indices 273-293, elevation 38.571429.
            new Vector3(-0.677085925f, 0.623489802f, 0.390915741f), // 273: yaw -60, elevation 38.571429
            new Vector3(-0.632514956f, 0.623489802f, 0.459549015f), // 274: yaw -54, elevation 38.571429
            new Vector3(-0.581014021f, 0.623489802f, 0.523147374f), // 275: yaw -48, elevation 38.571429
            new Vector3(-0.523147374f, 0.623489802f, 0.581014021f), // 276: yaw -42, elevation 38.571429
            new Vector3(-0.459549015f, 0.623489802f, 0.632514956f), // 277: yaw -36, elevation 38.571429
            new Vector3(-0.390915741f, 0.623489802f, 0.677085925f), // 278: yaw -30, elevation 38.571429
            new Vector3(-0.317999513f, 0.623489802f, 0.714238599f), // 279: yaw -24, elevation 38.571429
            new Vector3(-0.241599215f, 0.623489802f, 0.743565926f), // 280: yaw -18, elevation 38.571429
            new Vector3(-0.162551905f, 0.623489802f, 0.764746589f), // 281: yaw -12, elevation 38.571429
            new Vector3(-0.081723643f, 0.623489802f, 0.777548528f), // 282: yaw -6, elevation 38.571429
            new Vector3(0.000000000f, 0.623489802f, 0.781831482f), // 283: yaw 0, elevation 38.571429
            new Vector3(0.081723643f, 0.623489802f, 0.777548528f), // 284: yaw 6, elevation 38.571429
            new Vector3(0.162551905f, 0.623489802f, 0.764746589f), // 285: yaw 12, elevation 38.571429
            new Vector3(0.241599215f, 0.623489802f, 0.743565926f), // 286: yaw 18, elevation 38.571429
            new Vector3(0.317999513f, 0.623489802f, 0.714238599f), // 287: yaw 24, elevation 38.571429
            new Vector3(0.390915741f, 0.623489802f, 0.677085925f), // 288: yaw 30, elevation 38.571429
            new Vector3(0.459549015f, 0.623489802f, 0.632514956f), // 289: yaw 36, elevation 38.571429
            new Vector3(0.523147374f, 0.623489802f, 0.581014021f), // 290: yaw 42, elevation 38.571429
            new Vector3(0.581014021f, 0.623489802f, 0.523147374f), // 291: yaw 48, elevation 38.571429
            new Vector3(0.632514956f, 0.623489802f, 0.459549015f), // 292: yaw 54, elevation 38.571429
            new Vector3(0.677085925f, 0.623489802f, 0.390915741f), // 293: yaw 60, elevation 38.571429
            // Forward grid row 14: indices 294-314, elevation 45.
            new Vector3(-0.612372436f, 0.707106781f, 0.353553391f), // 294: yaw -60, elevation 45
            new Vector3(-0.572061403f, 0.707106781f, 0.415626938f), // 295: yaw -54, elevation 45
            new Vector3(-0.525482745f, 0.707106781f, 0.473146789f), // 296: yaw -48, elevation 45
            new Vector3(-0.473146789f, 0.707106781f, 0.525482745f), // 297: yaw -42, elevation 45
            new Vector3(-0.415626938f, 0.707106781f, 0.572061403f), // 298: yaw -36, elevation 45
            new Vector3(-0.353553391f, 0.707106781f, 0.612372436f), // 299: yaw -30, elevation 45
            new Vector3(-0.287606238f, 0.707106781f, 0.645974188f), // 300: yaw -24, elevation 45
            new Vector3(-0.218508012f, 0.707106781f, 0.672498512f), // 301: yaw -18, elevation 45
            new Vector3(-0.147015766f, 0.707106781f, 0.691654801f), // 302: yaw -12, elevation 45
            new Vector3(-0.073912785f, 0.707106781f, 0.703233176f), // 303: yaw -6, elevation 45
            new Vector3(0.000000000f, 0.707106781f, 0.707106781f), // 304: yaw 0, elevation 45
            new Vector3(0.073912785f, 0.707106781f, 0.703233176f), // 305: yaw 6, elevation 45
            new Vector3(0.147015766f, 0.707106781f, 0.691654801f), // 306: yaw 12, elevation 45
            new Vector3(0.218508012f, 0.707106781f, 0.672498512f), // 307: yaw 18, elevation 45
            new Vector3(0.287606238f, 0.707106781f, 0.645974188f), // 308: yaw 24, elevation 45
            new Vector3(0.353553391f, 0.707106781f, 0.612372436f), // 309: yaw 30, elevation 45
            new Vector3(0.415626938f, 0.707106781f, 0.572061403f), // 310: yaw 36, elevation 45
            new Vector3(0.473146789f, 0.707106781f, 0.525482745f), // 311: yaw 42, elevation 45
            new Vector3(0.525482745f, 0.707106781f, 0.473146789f), // 312: yaw 48, elevation 45
            new Vector3(0.572061403f, 0.707106781f, 0.415626938f), // 313: yaw 54, elevation 45
            new Vector3(0.612372436f, 0.707106781f, 0.353553391f), // 314: yaw 60, elevation 45
            // Side/rear belt: indices 315-329, elevation 0, ascending signed yaw.
            new Vector3(0.000000000f, 0.000000000f, -1.000000000f), // 315: yaw -180, elevation 0
            new Vector3(-0.258819045f, 0.000000000f, -0.965925826f), // 316: yaw -165, elevation 0
            new Vector3(-0.500000000f, 0.000000000f, -0.866025404f), // 317: yaw -150, elevation 0
            new Vector3(-0.707106781f, 0.000000000f, -0.707106781f), // 318: yaw -135, elevation 0
            new Vector3(-0.866025404f, 0.000000000f, -0.500000000f), // 319: yaw -120, elevation 0
            new Vector3(-0.965925826f, 0.000000000f, -0.258819045f), // 320: yaw -105, elevation 0
            new Vector3(-1.000000000f, 0.000000000f, 0.000000000f), // 321: yaw -90, elevation 0
            new Vector3(-0.965925826f, 0.000000000f, 0.258819045f), // 322: yaw -75, elevation 0
            new Vector3(0.965925826f, 0.000000000f, 0.258819045f), // 323: yaw 75, elevation 0
            new Vector3(1.000000000f, 0.000000000f, 0.000000000f), // 324: yaw 90, elevation 0
            new Vector3(0.965925826f, 0.000000000f, -0.258819045f), // 325: yaw 105, elevation 0
            new Vector3(0.866025404f, 0.000000000f, -0.500000000f), // 326: yaw 120, elevation 0
            new Vector3(0.707106781f, 0.000000000f, -0.707106781f), // 327: yaw 135, elevation 0
            new Vector3(0.500000000f, 0.000000000f, -0.866025404f), // 328: yaw 150, elevation 0
            new Vector3(0.258819045f, 0.000000000f, -0.965925826f), // 329: yaw 165, elevation 0
            // Upper ring: indices 330-341, ascending signed yaw.
            new Vector3(0.000000000f, 0.866025404f, -0.500000000f), // 330: yaw -180, elevation 60
            new Vector3(-0.250000000f, 0.866025404f, -0.433012702f), // 331: yaw -150, elevation 60
            new Vector3(-0.433012702f, 0.866025404f, -0.250000000f), // 332: yaw -120, elevation 60
            new Vector3(-0.500000000f, 0.866025404f, 0.000000000f), // 333: yaw -90, elevation 60
            new Vector3(-0.433012702f, 0.866025404f, 0.250000000f), // 334: yaw -60, elevation 60
            new Vector3(-0.250000000f, 0.866025404f, 0.433012702f), // 335: yaw -30, elevation 60
            new Vector3(0.000000000f, 0.866025404f, 0.500000000f), // 336: yaw 0, elevation 60
            new Vector3(0.250000000f, 0.866025404f, 0.433012702f), // 337: yaw 30, elevation 60
            new Vector3(0.433012702f, 0.866025404f, 0.250000000f), // 338: yaw 60, elevation 60
            new Vector3(0.500000000f, 0.866025404f, 0.000000000f), // 339: yaw 90, elevation 60
            new Vector3(0.433012702f, 0.866025404f, -0.250000000f), // 340: yaw 120, elevation 60
            new Vector3(0.250000000f, 0.866025404f, -0.433012702f), // 341: yaw 150, elevation 60
            // Lower ring: indices 342-353, ascending signed yaw.
            new Vector3(0.000000000f, -0.866025404f, -0.500000000f), // 342: yaw -180, elevation -60
            new Vector3(-0.250000000f, -0.866025404f, -0.433012702f), // 343: yaw -150, elevation -60
            new Vector3(-0.433012702f, -0.866025404f, -0.250000000f), // 344: yaw -120, elevation -60
            new Vector3(-0.500000000f, -0.866025404f, 0.000000000f), // 345: yaw -90, elevation -60
            new Vector3(-0.433012702f, -0.866025404f, 0.250000000f), // 346: yaw -60, elevation -60
            new Vector3(-0.250000000f, -0.866025404f, 0.433012702f), // 347: yaw -30, elevation -60
            new Vector3(0.000000000f, -0.866025404f, 0.500000000f), // 348: yaw 0, elevation -60
            new Vector3(0.250000000f, -0.866025404f, 0.433012702f), // 349: yaw 30, elevation -60
            new Vector3(0.433012702f, -0.866025404f, 0.250000000f), // 350: yaw 60, elevation -60
            new Vector3(0.500000000f, -0.866025404f, 0.000000000f), // 351: yaw 90, elevation -60
            new Vector3(0.433012702f, -0.866025404f, -0.250000000f), // 352: yaw 120, elevation -60
            new Vector3(0.250000000f, -0.866025404f, -0.433012702f), // 353: yaw 150, elevation -60
            // Steep forward/backward: indices 354-357; back/up, front/up, back/down, front/down.
            new Vector3(0.000000000f, 0.965925826f, -0.258819045f), // 354: yaw -180, elevation 75
            new Vector3(0.000000000f, 0.965925826f, 0.258819045f), // 355: yaw 0, elevation 75
            new Vector3(0.000000000f, -0.965925826f, -0.258819045f), // 356: yaw -180, elevation -75
            new Vector3(0.000000000f, -0.965925826f, 0.258819045f), // 357: yaw 0, elevation -75
            // Vertical poles: indices 358-359; straight up, then straight down.
            new Vector3(0.000000000f, 1.000000000f, 0.000000000f), // 358: yaw 0, elevation 90
            new Vector3(0.000000000f, -1.000000000f, 0.000000000f), // 359: yaw 0, elevation -90
        };

        // Codex code ends here

        // Awake runs once when the mod is loaded
        private void Awake()
        {
            Log = BepInEx.Logging.Logger.CreateLogSource("LidarMod");
            Log.LogInfo("Lidar Mod starting up");
            try{
                udpClient = new UdpClient();
                remoteEndPoint = new IPEndPoint(IPAddress.Parse("127.0.0.1"), 9002);
                Log.LogInfo("UDP Client Ready on port 9002");
            }
            catch (Exception e){
                Log.LogError($"Failed to setup UDP: {e.Message}");
            }
            Logger.LogInfo("Lidar Mod Loaded");
            // string message = "Hello\n";
            // byte[] sendBytes = Encoding.ASCII.GetBytes(message);
            // udpClient.Send(sendBytes, sendBytes.Length, remoteEndPoint);
        }


        private void FixedUpdate(){
            // string message = $"FixedUpdate {updateCounter}\n";
            // byte[] sendBytes = Encoding.ASCII.GetBytes(message);
            // udpClient.Send(sendBytes, sendBytes.Length, remoteEndPoint);
            updateCounter = updateCounter + 1;

            GameObject drone = GameObject.Find("DroneHUD_Custom(Clone)");
            if (drone == null){
                if(updateCounter % 100 == 0){
                    Log.LogInfo("LidarMod: Player not found..");
                }
                return;
            }
            else {
                if(updateCounter % 100 == 0){
                    Log.LogInfo($"LidarMod: Drone found at {drone.transform.position}");
                }
            }

            // Slow down the following loop
            if(updateCounter % physics_ticks_per_lidar_packet != 0){
                return;
            }

            float[] distances = new float[N_POINTS*2];
            Vector3 origin = drone.transform.position;

            if(use_jacob_rings==true){
                for (int i = 0; i < N_POINTS; i++)
                {
                    // Scan forward and to the sides
                    Vector3 direction = Quaternion.AngleAxis(i*(360f/N_POINTS), drone.transform.up) * drone.transform.forward;
                    if (Physics.Raycast(origin, direction, out RaycastHit hit, 50f))
                    {
                        distances[i] = hit.distance;
                    }
                    else
                    {
                        distances[i] = 50.0f;
                    }

                    // Scan up and to the sides
                    direction = Quaternion.AngleAxis(i*(360f/N_POINTS), drone.transform.forward) * drone.transform.up;
                    if (Physics.Raycast(origin, direction, out hit, 50f))
                    {
                        distances[N_POINTS+i] = hit.distance;
                    }
                    else
                    {
                        distances[N_POINTS+i] = 50.0f;
                    }
                }
            }
            else
            {
                // Codex code begins here: This should be the casting of every ray in each frame.
                Quaternion rotation = drone.transform.rotation;
                for (int i = 0; i < PrebakedRayDirections.Length; i++)
                {
                    Vector3 direction = rotation * PrebakedRayDirections[i];
                    distances[i] = Physics.Raycast(origin, direction, out RaycastHit hit, 50f)
                        ? hit.distance
                        : 50.0f;
                }

                // Codex code ends here
            }
            byte[] payload = new byte[distances.Length * 4];
            Buffer.BlockCopy(distances, 0, payload, 0, payload.Length);
            udpClient.Send(payload, payload.Length, remoteEndPoint);
            // string message = $"FixedUpdate {updateCounter}: {drone.transform.position} - {distances[0]}\n";
            // byte[] sendBytes = Encoding.ASCII.GetBytes(message);
            // udpClient.Send(sendBytes, sendBytes.Length, remoteEndPoint);
        }
    }
}
