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
        MyWaypointInfo pointTo;
        IMyGyro myGyro;
        IMyShipController myShipController;
        PID _pidYaw;
        PID _pidPitch;
        SEUtils _seu;
        const double ANGLE_TOLERANCE = 1d;

        public Program()
        {
            _seu = new SEUtils(this);
            _pidYaw = new PID(1.5f, 0, 0.5f, 0);
            _pidPitch = new PID(1.5f, 0, 0.5f, 0);
            MyWaypointInfo.TryParse(Me.CustomData, out pointTo);
            myGyro = GridTerminalSystem.GetBlockWithName("Gyroscope") as IMyGyro;
            myShipController = GridTerminalSystem.GetBlockWithName("Remote Control") as IMyShipController;
        }

        public void Save()
        {

        }

        public void Main(string argument, UpdateType updateSource)
        {
            Vector3D currentVelocity = myShipController.GetShipVelocities().LinearVelocity;
            Vector3D pointBetweenCurrentAndWP = pointTo.Coords + pointTo.Coords.To(myShipController.GetPosition() - currentVelocity) / Math.Max(1.2f, 3 - currentVelocity.Length());

            Vector3D waypointDirection = myShipController.WorldMatrix.Translation.To(pointBetweenCurrentAndWP);
            Vector3D localWaypointDirection = Vector3D.TransformNormal(waypointDirection, Matrix.Transpose(myShipController.WorldMatrix));
            localWaypointDirection.Normalize();
            double y, p;
            Vector3D.GetAzimuthAndElevation(localWaypointDirection, out y, out p);
            Echo(localWaypointDirection + " dir");
            Echo("Yaw: " + y + " Pitch: " + p);
            var timeStep = Runtime.TimeSinceLastRun.TotalSeconds;
            myGyro.Yaw = (float)_pidYaw.Control(-y, timeStep);
            myGyro.Pitch = (float)_pidPitch.Control(-p, timeStep);
        }
    }

    public class PID
    {
        public double Kp { get; set; } = 0;
        public double Ki { get; set; } = 0;
        public double Kd { get; set; } = 0;
        public double Value { get; private set; }

        double _timeStep = 0;
        double _inverseTimeStep = 0;
        double _errorSum = 0;
        double _lastError = 0;
        bool _firstRun = true;

        public PID(double kp, double ki, double kd, double timeStep)
        {
            Kp = kp;
            Ki = ki;
            Kd = kd;
            _timeStep = timeStep;
            _inverseTimeStep = 1 / _timeStep;
        }

        protected virtual double GetIntegral(double currentError, double errorSum, double timeStep)
        {
            return errorSum + currentError * timeStep;
        }

        public double Control(double error)
        {
            //Compute derivative term
            double errorDerivative = (error - _lastError) * _inverseTimeStep;

            if (_firstRun)
            {
                errorDerivative = 0;
                _firstRun = false;
            }

            //Get error sum
            _errorSum = GetIntegral(error, _errorSum, _timeStep);

            //Store this error as last error
            _lastError = error;

            //Construct output
            Value = Kp * error + Ki * _errorSum + Kd * errorDerivative;
            return Value;
        }

        public double Control(double error, double timeStep)
        {
            if (timeStep != _timeStep)
            {
                _timeStep = timeStep;
                _inverseTimeStep = 1 / _timeStep;
            }
            return Control(error);
        }

        public void Reset()
        {
            _errorSum = 0;
            _lastError = 0;
            _firstRun = true;
        }
    }
}
