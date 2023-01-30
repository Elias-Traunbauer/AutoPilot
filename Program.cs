using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
        List<Waypoint> waypoints;
        IMyGyro gyro;
        IMyShipController controller;
        PID _pidYaw;
        PID _pidPitch;
        PID _pidRoll;
        PID _pidX;
        PID _pidY;
        PID _pidZ;
        SEUtils _seu;
        const double POS_TOLERANCE = 0.2d;
        const double ANGLE_TOLERANCE = 0.1d;
        IMyTextSurface cockpitLCD;
        List<MyTuple<Vector3I, IMyThrust>> thrusters;
        Dictionary<Base6Directions.Direction, Vector3I> directions = new Dictionary<Base6Directions.Direction, Vector3I>() {
            { Base6Directions.Direction.Forward, Vector3I.Forward },
            { Base6Directions.Direction.Left, Vector3I.Left },
            { Base6Directions.Direction.Right, Vector3I.Right },
            { Base6Directions.Direction.Down, Vector3I.Down },
            { Base6Directions.Direction.Up, Vector3I.Up },
            { Base6Directions.Direction.Backward, Vector3I.Backward }
        };

        public Program()
        {
            _seu = new SEUtils(this);
            _pidYaw = new PID(.3f, 0, 0.07f, 0);
            _pidPitch = new PID(.3f, 0, 0.07f, 0);
            _pidRoll = new PID(.3f, 0, 0.07f, 0);
            _pidX = new PID(.3f, 0, 0.28f, 0);
            _pidY = new PID(.3f, 0, 0.28f, 0);
            _pidZ = new PID(.3f, 0, 0.28f, 0);
            thrusters = new List<MyTuple<Vector3I, IMyThrust>>();

            waypoints = new List<Waypoint>();
            gyro = GridTerminalSystem.GetBlockWithName("Gyroscope") as IMyGyro;
            controller = GridTerminalSystem.GetBlockWithName("Cockpit") as IMyShipController;
            cockpitLCD = (controller as IMyCockpit).GetSurface(0);

            var controllerDirection = controller.Orientation;
            List<IMyThrust> thrusts = new List<IMyThrust>();
            GridTerminalSystem.GetBlocksOfType(thrusts, _seu.IsInGrid);
            foreach (var item in thrusts)
            {
                var localDir = controllerDirection.TransformDirection(Base6Directions.GetOppositeDirection(item.Orientation.Forward));
                thrusters.Add(new MyTuple<Vector3I, IMyThrust>(directions[localDir], item));
            }
            _seu.StartCoroutine(Autopilot());
        }
        string argument = "";

        public void Save()
        {

        }

        public void Main(string argument, UpdateType updateSource)
        {
            this.argument = argument;
            if (!_seu.RuntimeUpdate(argument, updateSource)) return;
        }

        public double LookAtStep(Vector3D lookAt, Vector3D rightLookAt)
        {
            Vector3D waypointDirection = controller.GetPosition().To(lookAt);
            Vector3D rollDirection = controller.GetPosition().To(controller.GetPosition() + rightLookAt);
            Vector3D localWaypointDirection = Vector3D.TransformNormal(waypointDirection, Matrix.Transpose(controller.WorldMatrix));
            Vector3D localRollDirection = Vector3D.TransformNormal(rollDirection, Matrix.Transpose(controller.WorldMatrix));
            localWaypointDirection.Normalize();
            localRollDirection.Normalize();
            double y, p, yr, pr;
            Vector3D.GetAzimuthAndElevation(localWaypointDirection, out y, out p);
            Vector3D.GetAzimuthAndElevation(localRollDirection, out yr, out pr);
            Echo(localWaypointDirection + " dir");
            Echo("Yaw: " + y + " Pitch: " + p);
            var timeStep = Runtime.TimeSinceLastRun.TotalSeconds;
            gyro.Yaw = (float)_pidYaw.Control(-y, timeStep);
            gyro.Pitch = (float)_pidPitch.Control(-p, timeStep);
            gyro.Roll = (float)_pidRoll.Control(-pr, timeStep);
            return y + p + pr;
        }

        IEnumerator Autopilot()
        {
            yield return new WaitForNextUpdate();
            Waypoint? currentWP = null;
            // 0 = idle, 1 = autopilot, 2 = record
            int state = 0;
            int wpIndex = 0;
            while (true)
            {
                if (!string.IsNullOrEmpty(argument))
                {
                    switch (argument)
                    {
                        case "idle":
                            gyro.Yaw = 0;
                            gyro.Pitch = 0;
                            state = 0;
                            break;

                        case "record":
                            state = 2;
                            break;

                        case "takeState":
                            if (state == 2)
                            {
                                waypoints.Add(new Waypoint() { position = controller.GetPosition(), rotation = controller.WorldMatrix.Forward, rightRot = controller.WorldMatrix.Right });
                            }
                            break;

                        case "pilot":
                            _pidPitch.Reset();
                            _pidYaw.Reset();
                            gyro.Yaw = 0;
                            gyro.Pitch = 0;
                            if (state != 1)
                            {
                                wpIndex = 0;
                            }
                            state = 1;
                            break;

                        default:
                            break;
                    }
                }
                
                if (state == 1)
                {
                    var timeStep = Runtime.TimeSinceLastRun.TotalSeconds;

                    if (currentWP == null)
                    {
                        currentWP = waypoints.First();
                    }

                    Vector3D pos = ((Waypoint)currentWP).position;
                    var angleError = LookAtStep(controller.GetPosition() + ((Waypoint)currentWP).rotation, ((Waypoint)currentWP).rightRot);

                    if (controller.GetPosition().To(pos).Length() < POS_TOLERANCE && angleError < ANGLE_TOLERANCE)
                    {
                        wpIndex++;
                        if (wpIndex >= waypoints.Count)
                        {
                            wpIndex = 0;
                        }
                        currentWP = waypoints[wpIndex];
                        pos = ((Waypoint)currentWP).position;
                        _pidX.Reset();
                        _pidY.Reset();
                        _pidZ.Reset();
                        _pidPitch.Reset();
                        _pidYaw.Reset();
                    }

                    Vector3D pidError = controller.GetPosition() - pos;
                    if (pidError.Length() < 4)
                    {
                        pidError *= Math.Max(4 - pidError.Length(), 1.5f);
                    }
                    Vector3D pidRes = new Vector3D(_pidX.Control(pidError.X, timeStep), _pidY.Control(pidError.Y, timeStep), _pidZ.Control(pidError.Z, timeStep));

                    foreach (var item in thrusters)
                    {
                        Vector3D localForce = Vector3D.TransformNormal(pidRes, MatrixD.Transpose(item.Item2.WorldMatrix));
                        Vector3D forceForThisVector = Vector3D.Forward * localForce;
                        if (forceForThisVector.Min() >= 0)
                        {
                            float force = (float)forceForThisVector.Length() / 10;
                            item.Item2.ThrustOverridePercentage = force;
                        }
                        else
                        {
                            item.Item2.ThrustOverridePercentage = 0;
                        }
                    }
                }
                
                cockpitLCD.WriteText(state == 0 ? "idle" : state == 1 ? "pilot" : "recording");
                cockpitLCD.WriteText("\nWP Count: " + waypoints.Count(), true);
                cockpitLCD.ContentType = ContentType.TEXT_AND_IMAGE;
                yield return new WaitForNextUpdate();
            }
        }
    }

    public struct Waypoint
    {
        public Vector3D position;
        public Vector3D rotation;
        public Vector3D rightRot;
    }

    public class PID
    {
        public double Proportinal { get; set; } = 0;
        public double Integral { get; set; } = 0;
        public double Derivative { get; set; } = 0;
        public double Value { get; private set; }

        double timeStep = 0;
        double inverseTimestamp = 0;
        double sumOfErrors = 0;
        double previousError = 0;
        bool firstControl = true;

        public PID(double p, double i, double d, double interval)
        {
            Proportinal = p;
            Integral = i;
            Derivative = d;
            this.timeStep = interval;
            inverseTimestamp = 1 / this.timeStep;
        }

        protected virtual double GetIntegral(double error, double errorSum, double timeStep)
        {
            return errorSum + error * timeStep;
        }

        private double Control(double error)
        {
            double errorDerivative = (error - previousError) * inverseTimestamp;

            if (firstControl)
            {
                errorDerivative = 0;
                firstControl = false;
            }
            sumOfErrors = GetIntegral(error, sumOfErrors, timeStep);
            previousError = error;
            Value = Proportinal * error + Integral * sumOfErrors + Derivative * errorDerivative;
            return Value;
        }

        public double Control(double error, double timeStep)
        {
            if (timeStep != this.timeStep)
            {
                this.timeStep = timeStep;
                inverseTimestamp = 1 / this.timeStep;
            }
            return Control(error);
        }

        public void Reset()
        {
            sumOfErrors = 0;
            previousError = 0;
            firstControl = true;
        }
    }
}
