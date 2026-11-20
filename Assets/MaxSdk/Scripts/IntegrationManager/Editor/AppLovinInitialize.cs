//
//  MaxInitialization.cs
//  AppLovin MAX Unity Plugin
//
//  Created by Thomas So on 5/24/19.
//  Copyright © 2019 AppLovin. All rights reserved.
//

using System.Collections.Generic;
using System.IO;
using UnityEditor;

namespace AppLovinMax.Scripts.IntegrationManager.Editor
{
    [InitializeOnLoad]
    public class AppLovinInitialize
    {
        private class ObsoleteNetwork
        {
            private readonly string name;
            public List<string> Packages { get; private set; }

            public ObsoleteNetwork(string name, params string[] packages)
            {
                this.name = name;
                Packages = new List<string>(packages);
            }

            public string GetNetworkPath()
            {
                return "MaxSdk/Mediation/" + name;
            }
        }

        private static readonly List<ObsoleteNetwork> ObsoleteNetworks = new List<ObsoleteNetwork>
        {
            new ObsoleteNetwork("AdColony"),
            new ObsoleteNetwork("Criteo"),
            new ObsoleteNetwork("CSJ", "com.applovin.mediation.adapters.csj.ios"),
            new ObsoleteNetwork("HyprMX", "com.applovin.mediation.adapters.hyprmx.android", "com.applovin.mediation.adapters.hyprmx.ios"),
            new ObsoleteNetwork("Maio", "com.applovin.mediation.adapters.maio.android", "com.applovin.mediation.adapters.maio.ios"),
            new ObsoleteNetwork("MyTarget", "com.applovin.mediation.adapters.mytarget.android", "com.applovin.mediation.adapters.mytarget.ios"),
            new ObsoleteNetwork("Nend"),
            new ObsoleteNetwork("Snap"),
            new ObsoleteNetwork("Tapjoy"),
            new ObsoleteNetwork("TencentGDT", "com.applovin.mediation.adapters.tencentgdt.ios"),
            new ObsoleteNetwork("VerizonAds"),
            new ObsoleteNetwork("VoodooAds")
        };

        private static readonly List<string> ObsoleteFileExportPathsToDelete = new List<string>
        {
            // The `MaxSdk/Scripts/Editor` folder contents have been moved into `MaxSdk/Scripts/IntegrationManager/Editor`.
            "MaxSdk/Scripts/Editor",
            "MaxSdk/Scripts/Editor.meta",

            // The `EventSystemChecker` has been renamed to `MaxEventSystemChecker`.
            "MaxSdk/Scripts/EventSystemChecker.cs",
            "MaxSdk/Scripts/EventSystemChecker.cs.meta",

            // Google AdMob adapter pre/post process scripts. The logic has been migrated to the main plugin.
            "MaxSdk/Mediation/Google/Editor/MaxGoogleInitialize.cs",
            "MaxSdk/Mediation/Google/Editor/MaxGoogleInitialize.cs.meta",
            "MaxSdk/Mediation/Google/Editor/MaxMediationGoogleUtils.cs",
            "MaxSdk/Mediation/Google/Editor/MaxMediationGoogleUtils.cs.meta",
            "MaxSdk/Mediation/Google/Editor/PostProcessor.cs",
            "MaxSdk/Mediation/Google/Editor/PostProcessor.cs.meta",
            "MaxSdk/Mediation/Google/Editor/PreProcessor.cs",
            "MaxSdk/Mediation/Google/Editor/PreProcessor.cs.meta",
            "MaxSdk/Mediation/Google/Editor/MaxSdk.Mediation.Google.Editor.asmdef",
            "MaxSdk/Mediation/Google/MaxSdk.Mediation.Google.Editor.asmdef.meta",
            "Plugins/Android/MaxMediationGoogle.androidlib",
            "Plugins/Android/MaxMediationGoogle.androidlib.meta",

            // Google Ad Manager adapter pre/post process scripts. The logic has been migrated to the main plugin.
            "MaxSdk/Mediation/GoogleAdManager/Editor/MaxGoogleAdManagerInitialize.cs",
            "MaxSdk/Mediation/GoogleAdManager/Editor/MaxGoogleAdManagerInitialize.cs.meta",
            "MaxSdk/Mediation/GoogleAdManager/Editor/PostProcessor.cs",
            "MaxSdk/Mediation/GoogleAdManager/Editor/PostProcessor.cs.meta",
            "MaxSdk/Mediation/GoogleAdManager/Editor/MaxSdk.Mediation.GoogleAdManager.Editor.asmdef",
            "MaxSdk/Mediation/GoogleAdManager/Editor/MaxSdk.Mediation.GoogleAdManager.Editor.asmdef.meta",
            "Plugins/Android/MaxMediationGoogleAdManager.androidlib",
            "Plugins/Android/MaxMediationGoogleAdManager.androidlib.meta",

            // The `VariableService` has been removed.
            "MaxSdk/Scripts/MaxVariableServiceAndroid.cs",
            "MaxSdk/Scripts/MaxVariableServiceAndroid.cs.meta",
            "MaxSdk/Scripts/MaxVariableServiceiOS.cs",
            "MaxSdk/Scripts/MaxVariableServiceiOS.cs.meta",
            "MaxSdk/Scripts/MaxVariableServiceUnityEditor.cs",
            "MaxSdk/Scripts/MaxVariableServiceUnityEditor.cs.meta",

            // The `MaxSdk/Scripts/Editor` folder contents have been moved into `MaxSdk/Scripts/IntegrationManager/Editor`.
            "MaxSdk/Version.md",
            "MaxSdk/Version.md.meta",

            // The alert_icon.png has been renamed to error_icon.png.
            "MaxSdk/Resources/Images/alert_icon.png",
            "MaxSdk/Resources/Images/alert_icon.png.meta",

            // `TargetingData` has been removed and we no longer set `UserSegment` through the Unity Plugin.
            "MaxSdk/Scripts/MaxUserSegment.cs",
            "MaxSdk/Scripts/MaxUserSegment.cs.meta",
            "MaxSdk/Scripts/MaxTargetingData.cs",
            "MaxSdk/Scripts/MaxTargetingData.cs.meta"
        };

        static AppLovinInitialize()
        {
            // Don't run obsolete file cleanup logic when entering play mode.
            if (EditorApplication.isPlayingOrWillChangePlaymode) return;

#if UNITY_IOS
            // Check that the publisher is targeting iOS 9.0+
            if (!PlayerSettings.iOS.targetOSVersionString.StartsWith("9.") && !PlayerSettings.iOS.targetOSVersionString.StartsWith("1"))
            {
                MaxSdkLogger.UserError("Detected iOS project version less than iOS 9 - The AppLovin MAX SDK WILL NOT WORK ON < iOS9!!!");
            }
#endif
            if (AppLovinIntegrationManager.IsPluginInPackageManager)
            {
                var appLovinManifest = AppLovinUpmManifest.Load();
                if (RemoveObsoleteNetworkPackages(appLovinManifest))
                {
                    appLovinManifest.Save();
                    AppLovinUpmPackageManager.ResolvePackageManager();
                    MaxSdkLogger.UserDebug("Obsolete networks removed.");
                }
            }
            else
            {
                var filesRemoved = RemoveObsoleteFiles();
                var networksRemoved = RemoveObsoleteNetworks();

                if (filesRemoved || networksRemoved)
                {
                    // Refresh UI
                    AssetDatabase.Refresh();
                    MaxSdkLogger.UserDebug("Obsolete networks and files removed.");
                }
            }

            AppLovinAutoUpdater.Update();
        }

        /// <summary>
        /// Removes obsolete files from the Unity project.
        /// </summary>
        /// <returns>True if any changes were made, otherwise returns false</returns>
        private static bool RemoveObsoleteFiles()
        {
            var changesMade = false;
            foreach (var obsoleteFileExportPathToDelete in ObsoleteFileExportPathsToDelete)
            {
                var pathToDelete = MaxSdkUtils.GetAssetPathForExportPath(obsoleteFileExportPathToDelete);
                if (CheckExistence(pathToDelete))
                {
                    MaxSdkLogger.UserDebug("Deleting obsolete file '" + pathToDelete + "' that is no longer needed.");
                    FileUtil.DeleteFileOrDirectory(pathToDelete);
                    changesMade = true;
                }
            }

            return changesMade;
        }

        /// <summary>
        /// Removes obsolete networks from the Unity project.
        /// </summary>
        /// <returns>True if any changes were made, otherwise returns false</returns>
        private static bool RemoveObsoleteNetworks()
        {
            var changesMade = false;
            var pluginParentDir = AppLovinIntegrationManager.PluginParentDirectory;
            // Check if any obsolete networks are installed
            foreach (var obsoleteNetwork in ObsoleteNetworks)
            {
                var networkDir = Path.Combine(pluginParentDir, obsoleteNetwork.GetNetworkPath());
                if (CheckExistence(networkDir))
                {
                    MaxSdkLogger.UserDebug("Deleting obsolete network " + obsoleteNetwork + " from path " + networkDir + "...");
                    FileUtil.DeleteFileOrDirectory(networkDir);
                    FileUtil.DeleteFileOrDirectory(networkDir + ".meta");
                    changesMade = true;
                }
            }

            return changesMade;
        }

        /// <summary>
        /// Removes obsolete network packages from the Unity project.
        /// </summary>
        /// <returns>True if any changes were made, otherwise returns false</returns>
        private static bool RemoveObsoleteNetworkPackages(AppLovinUpmManifest appLovinManifest)
        {
            var changesMade = false;
            foreach (var obsoleteNetwork in ObsoleteNetworks)
            {
                foreach (var packageName in obsoleteNetwork.Packages)
                {
                    if (appLovinManifest.RemovePackageDependency(packageName))
                    {
                        MaxSdkLogger.UserDebug("Uninstalling obsolete network package" + packageName);
                        changesMade = true;
                    }
                }
            }

            return changesMade;
        }

        private static bool CheckExistence(string location)
        {
            return File.Exists(location) ||
                   Directory.Exists(location) ||
                   (location.EndsWith("/*") && Directory.Exists(Path.GetDirectoryName(location)));
        }
    }
}
