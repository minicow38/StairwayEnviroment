// Unity 2022.3 Editor only: place in Assets/Editor/ProfilerCaptureCsvExporter.cs.
// Load a saved .data file in Window > Analysis > Profiler first.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEditor;
using UnityEditor.Profiling;
using UnityEditorInternal;
using UnityEngine;

public static class ProfilerCaptureCsvExporter
{
    private sealed class SampleStat
    {
        public int Calls;
        public double TotalMs;
        public double MaxMs;
        public long GcBytes;
        public void Add(double ms, long gcBytes)
        {
            Calls++;
            TotalMs += ms;
            if (ms > MaxMs) MaxMs = ms;
            GcBytes += gcBytes;
        }
    }

    [MenuItem("Tools/Profiler/Export Loaded Capture to CSV")]
    public static void Export()
    {
        int first = ProfilerDriver.firstFrameIndex;
        int last = ProfilerDriver.lastFrameIndex;
        if (first < 0 || last < first)
        {
            EditorUtility.DisplayDialog("Profiler CSV Export", "Profilerウィンドウで .data ファイルを Load してから実行してください。", "OK");
            return;
        }

        string framesPath = EditorUtility.SaveFilePanel("Save profiler frame summary", Application.dataPath, "bugDelay_frames", "csv");
        if (string.IsNullOrEmpty(framesPath)) return;
        string samplesPath = Path.Combine(Path.GetDirectoryName(framesPath), Path.GetFileNameWithoutExtension(framesPath) + "_samples.csv");
        int exported = 0;
        try
        {
            using (var frames = new StreamWriter(framesPath, false, new UTF8Encoding(true)))
            using (var samples = new StreamWriter(samplesPath, false, new UTF8Encoding(true)))
            {
                frames.WriteLine("Frame,FrameTimeMs,FPS,MainThread,PlayerLoopMs,FixedPhysicsMs,FixedPhysicsCalls,PhysicsSimulateMs,PhysicsSimulateCalls,GcAllocBytes,EqualizerMs,WeaverMs");
                samples.WriteLine("Frame,FrameTimeMs,MarkerName,Calls,InclusiveTotalMs,MaxSingleSampleMs,GcAllocBytes");
                // Unity profiler frame numbers can include gaps; use the valid frame range.
                for (int frame = first; frame <= last; frame++)
                {
                    if ((frame - first) % 20 == 0)
                        EditorUtility.DisplayProgressBar("Exporting Profiler Capture", "Frame " + frame + " / " + last, (float)(frame - first) / Math.Max(1, last - first + 1));

                    // Thread index 0 is the main thread in standard Unity player captures.
                    using (RawFrameDataView view = ProfilerDriver.GetRawFrameDataView(frame, 0))
                    {
                        if (!view.valid) continue;
                        var stats = new Dictionary<string, SampleStat>(StringComparer.Ordinal);
                        int gcMarkerId = view.GetMarkerId("GC.Alloc");
                        long gcTotal = 0;
                        for (int i = 0; i < view.sampleCount; ++i)
                        {
                            string name = view.GetSampleName(i);
                            if (string.IsNullOrEmpty(name)) continue;
                            double duration = view.GetSampleTimeMs(i);
                            long gc = 0;
                            if (gcMarkerId != FrameDataView.invalidMarkerId && view.GetSampleMarkerId(i) == gcMarkerId)
                            {
                                try { gc = view.GetSampleMetadataAsLong(i, 0); }
                                catch (Exception) { gc = 0; }
                                gcTotal += gc;
                            }
                            SampleStat s;
                            if (!stats.TryGetValue(name, out s))
                            {
                                s = new SampleStat();
                                stats.Add(name, s);
                            }
                            s.Add(duration, gc);
                        }

                        double frameMs = view.frameTimeMs;
                        double fps = view.frameFps;
                        double player = Total(stats, "PlayerLoop");
                        double fixedMs = Total(stats, "FixedUpdate.PhysicsFixedUpdate");
                        int fixedCalls = Calls(stats, "FixedUpdate.PhysicsFixedUpdate");
                        double simulation = Total(stats, "Physics.Simulate");
                        int simCalls = Calls(stats, "Physics.Simulate");
                        double equalizerMs = SumNameContains(stats, "ExampleBallVisualEqualizer");
                        double weaverMs = SumNameContains(stats, "IndependentFiveSplineWeaver");

                        frames.WriteLine(string.Join(",", new string[] {
                            frame.ToString(), N(frameMs), N(fps), Csv(view.threadName), N(player), N(fixedMs),
                            fixedCalls.ToString(), N(simulation), simCalls.ToString(), gcTotal.ToString(),
                            N(equalizerMs), N(weaverMs)
                        }));
                        foreach (var pair in stats)
                        {
                            SampleStat s = pair.Value;
                            samples.WriteLine(string.Join(",", new string[] {
                                frame.ToString(), N(frameMs), Csv(pair.Key), s.Calls.ToString(),
                                N(s.TotalMs), N(s.MaxMs), s.GcBytes.ToString()
                            }));
                        }
                        exported++;
                    }
                }
            }
            EditorUtility.DisplayDialog("Profiler CSV Export", "Exported " + exported + " frames.\n\n" + framesPath + "\n" + samplesPath + "\n\nUpload both CSV files for comparison.", "OK");
            Debug.Log("[Profiler CSV Export] " + exported + " frames: " + framesPath + " | " + samplesPath);
        }
        catch (Exception ex)
        {
            Debug.LogException(ex);
            EditorUtility.DisplayDialog("Profiler CSV Export failed", ex.Message, "OK");
        }
        finally
        {
            EditorUtility.ClearProgressBar();
        }
    }

    private static double Total(Dictionary<string, SampleStat> stats, string name)
    {
        SampleStat s;
        return stats.TryGetValue(name, out s) ? s.TotalMs : 0;
    }

    private static int Calls(Dictionary<string, SampleStat> stats, string name)
    {
        SampleStat s;
        return stats.TryGetValue(name, out s) ? s.Calls : 0;
    }

    private static double SumNameContains(Dictionary<string, SampleStat> stats, string text)
    {
        double sum = 0;
        foreach (var pair in stats)
            if (pair.Key.IndexOf(text, StringComparison.OrdinalIgnoreCase) >= 0)
                sum += pair.Value.TotalMs;
        return sum;
    }

    private static string N(double value) { return value.ToString("F4", CultureInfo.InvariantCulture); }
    private static string Csv(string value) { return "\"" + (value ?? "").Replace("\"", "\"\"") + "\""; }
}
