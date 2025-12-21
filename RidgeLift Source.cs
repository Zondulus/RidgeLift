using System;
using System.Collections;
using UnityEngine;
using WindAPI;

namespace RidgeLift
{
    [KSPAddon(KSPAddon.Startup.Flight, once: false)]
    public class RidgeLift : MonoBehaviour, IWindProvider
    {
        // --- Configuration ---
        private float probeDistance = 750f;
        private float liftMultiplier = 2.0f;
        private float maxCeiling = 2500f;
        private float rampHeight = 500f;
        private float smoothingSpeed = 5.0f;

        // The altitude at which full effects begin (fades out below this)
        private float lowAltCutoff = 50f;
        private bool debugMode = true;

        // --- State ---
        private Vector3 smoothedLiftVector = Vector3.zero;
        private Vector3 targetLiftVector = Vector3.zero;
        private double lastCalculatedSlope = 0;

        private int frameCount = 0;
        private const int UPDATE_INTERVAL = 5;

        // UI & Debug
        private float msgTimer = 0f;
        private const float MSG_INTERVAL = 2f;

        // --- IWindProvider Implementation ---
        public string ProviderID => "RidgeLift";

        public Vector3 GetWind(CelestialBody body, Part part, Vector3 position)
        {
            if (smoothedLiftVector == Vector3.zero) return Vector3.zero;

            // 1. Direct Part Check
            if (part != null)
            {
                if (part.vessel == FlightGlobals.ActiveVessel)
                    return smoothedLiftVector;
            }

            // 2. Proximity Check (For Null parts or nearby vessels)
            if (FlightGlobals.ActiveVessel != null)
            {
                // Optimization: sqrMagnitude avoids slow square root calc
                float distSqr = (float)(position - FlightGlobals.ActiveVessel.GetWorldPos3D()).sqrMagnitude;
                if (distSqr < 4000000f) // 2000m range squared
                {
                    return smoothedLiftVector;
                }
            }

            return Vector3.zero;
        }

        // --- Lifecycle ---

        public void Start()
        {
            LoadConfig();
            StartCoroutine(RegisterWithAPI());
        }

        private IEnumerator RegisterWithAPI()
        {
            while (WindManager.Instance == null) yield return new WaitForSeconds(0.5f);
            WindManager.Instance.RegisterProvider(this);
            if (debugMode) Debug.Log("[RidgeLift] Registered with WindAPI.");
        }

        public void OnDestroy()
        {
            if (WindManager.Instance != null)
                WindManager.Instance.DeregisterProvider(this);
        }

        // --- Physics Loop ---

        public void FixedUpdate()
        {
            if (!FlightGlobals.ready || FlightGlobals.ActiveVessel == null)
            {
                targetLiftVector = Vector3.zero;
                smoothedLiftVector = Vector3.zero;
                return;
            }

            smoothedLiftVector = Vector3.Lerp(smoothedLiftVector, targetLiftVector, Time.fixedDeltaTime * smoothingSpeed);

            frameCount++;
            if (frameCount >= UPDATE_INTERVAL)
            {
                frameCount = 0;
                CalculateRidgeLiftTarget(FlightGlobals.ActiveVessel);
            }
        }

        public void Update()
        {
            if (!debugMode || FlightGlobals.ActiveVessel == null) return;

            msgTimer += Time.deltaTime;
            if (msgTimer > MSG_INTERVAL)
            {
                msgTimer = 0f;
                if (smoothedLiftVector.sqrMagnitude > 1.0f)
                {
                    string type = lastCalculatedSlope > 0 ? "Lift" : "Sink";
                    ScreenMessages.PostScreenMessage($"[Ridge] {type}: {smoothedLiftVector.magnitude:F1} m/s", 1.5f, ScreenMessageStyle.UPPER_CENTER);
                }
            }
        }

        // --- Core Logic ---

        private void CalculateRidgeLiftTarget(Vessel v)
        {
            CelestialBody body = v.mainBody;

            // Basic sanity checks
            if (!body.atmosphere || v.altitude > 30000)
            {
                targetLiftVector = Vector3.zero;
                return;
            }

            Vector3 vesselPos = v.GetWorldPos3D();

            // 1. Get Base Wind, ignore if insignificant
            Vector3 baseWind = WindManager.Instance.GetWindAtLocation(body, v.rootPart, vesselPos, ignoreProvider: this);
            float windSpeed = baseWind.magnitude;

            if (windSpeed < 1.0f)
            {
                targetLiftVector = Vector3.zero;
                return;
            }

            // 2. Determine Upwind Coordinates
            Vector3 windDir = baseWind.normalized;
            Vector3d upwindPos = (Vector3d)vesselPos - (Vector3d)(windDir * probeDistance);

            // 3. Sample Terrain
            double vLat = v.latitude;
            double vLon = v.longitude;
            double uLat = body.GetLatitude(upwindPos);
            double uLon = body.GetLongitude(upwindPos);

            double hVessel = body.TerrainAltitude(vLat, vLon);
            double hUpwind = body.TerrainAltitude(uLat, uLon);

            // Clamp ocean to 0
            if (hVessel < 0) hVessel = 0;
            if (hUpwind < 0) hUpwind = 0;

            // 4. Calculate Slope
            double terrainRise = hVessel - hUpwind;
            double slope = terrainRise / probeDistance;
            lastCalculatedSlope = slope;

            // 5. Calculate Vertical Speed
            double verticalSpeed = windSpeed * slope * liftMultiplier;
            double maxVert = windSpeed * 2.0;
            verticalSpeed = Math.Max(-maxVert, Math.Min(verticalSpeed, maxVert));

            // --- 6. Altitude Attenuation ---
            double agl = v.altitude - hVessel;
            if (agl < 0) agl = 0;

            // Factor A: Low Altitude Cutoff
            // 0.0 at 10m, 1.0 at 50m (or whatever 'lowAltCutoff' is set to)
            // This ensures no effect on low approach or ground
            double lowAltFactor = 1.0;
            if (agl < lowAltCutoff)
            {
                // We leave a 10m buffer so it is totally dead when wheels are on ground
                double buffer = 10.0;
                if (agl <= buffer)
                {
                    lowAltFactor = 0.0;
                }
                else
                {
                    lowAltFactor = (agl - buffer) / (lowAltCutoff - buffer);
                }
            }

            // Factor B: High Altitude Ceiling
            double highAltFactor = 0.0;
            if (agl < rampHeight)
            {
                highAltFactor = 1.0;
            }
            else if (agl < maxCeiling)
            {
                double range = maxCeiling - rampHeight;
                double current = agl - rampHeight;
                highAltFactor = 1.0 - (current / range);
            }

            // Apply factors
            verticalSpeed *= highAltFactor;
            verticalSpeed *= lowAltFactor;

            // 7. Apply Result
            Vector3 upVector = (vesselPos - body.position).normalized;
            targetLiftVector = upVector * (float)verticalSpeed;
        }

        private void LoadConfig()
        {
            ConfigNode[] nodes = GameDatabase.Instance.GetConfigNodes("RIDGELIFT_SETTINGS");
            if (nodes != null && nodes.Length > 0)
            {
                ConfigNode n = nodes[0];
                if (n.HasValue("probeDistance")) float.TryParse(n.GetValue("probeDistance"), out probeDistance);
                if (n.HasValue("liftMultiplier")) float.TryParse(n.GetValue("liftMultiplier"), out liftMultiplier);
                if (n.HasValue("maxCeiling")) float.TryParse(n.GetValue("maxCeiling"), out maxCeiling);
                if (n.HasValue("rampHeight")) float.TryParse(n.GetValue("rampHeight"), out rampHeight);
                if (n.HasValue("smoothingSpeed")) float.TryParse(n.GetValue("smoothingSpeed"), out smoothingSpeed);
                if (n.HasValue("lowAltCutoff")) float.TryParse(n.GetValue("lowAltCutoff"), out lowAltCutoff);
                if (n.HasValue("debugMode")) bool.TryParse(n.GetValue("debugMode"), out debugMode);
            }
        }
    }
}