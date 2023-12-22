using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEngine;

namespace OctopusController
{
    public enum TentacleMode { LEG, TAIL, TENTACLE };

    public class MyOctopusController
    {

        MyTentacleController[] _tentacles = new MyTentacleController[4];

        Transform _currentRegion;
        Transform _target;
        Transform _ball;

        Transform[] _randomTargets;// = new Transform[4];
        Vector3[] _targetOffsets;
        bool[] _targetBall;

        float _twistMin, _twistMax;
        float _swingMin, _swingMax;

        #region public methods
        //DO NOT CHANGE THE PUBLIC METHODS!!

        public float TwistMin { set => _twistMin = value; }
        public float TwistMax { set => _twistMax = value; }
        public float SwingMin { set => _swingMin = value; }
        public float SwingMax { set => _swingMax = value; }


        public void TestLogging(string objectName)
        {


            Debug.Log("hello, I am initializing my Octopus Controller in object " + objectName);


        }

        public void Init(Transform[] tentacleRoots, Transform[] randomTargets)
        {
            Transform[] actualTentacleRoots = new Transform[tentacleRoots.Length];
            for (int i = 0; i < tentacleRoots.Length; i++)
            {
                actualTentacleRoots[i] = tentacleRoots[i].GetChild(0).GetChild(0);
            }
            tentacleRoots = actualTentacleRoots;
            _tentacles = new MyTentacleController[tentacleRoots.Length];
            _targetOffsets = new Vector3[tentacleRoots.Length];

            _randomTargets = randomTargets;
            SetIdle();

            // foreach (Transform t in tentacleRoots)
            for (int i = 0; i < tentacleRoots.Length; i++)
            {

                _tentacles[i] = new MyTentacleController();
                _tentacles[i].LoadTentacleJoints(tentacleRoots[i], TentacleMode.TENTACLE);
                _targetOffsets[i] = _tentacles[i].Bones[_tentacles[i].Bones.Length - 1].position - randomTargets[i].position;
                //TODO: initialize any variables needed in ccd
            }
            //TODO: use the regions however you need to make sure each tentacle stays in its region

        }


        public void NotifyTarget(Transform target, Transform region)
        {
            _currentRegion = region;
            _target = target;
        }

        public void NotifyStop()
        {
            SetIdle();
        }


        public void UpdateTentacles()
        {
            update_ccd();
        }

        public void StopBall(Transform ballTransform, Vector3 stopPosition)
        {
            _ball = ballTransform;
            int closest = 0;
            for (int i = 1; i < _randomTargets.Length; i++)
            {
                if (Vector3.Distance(stopPosition, _randomTargets[i].position) < Vector3.Distance(stopPosition, _randomTargets[closest].position))
                {
                    closest = i;
                }
            }
            _targetBall[closest] = true;

        }

        public void SetIdle()
        {
            _targetBall = new bool[_randomTargets.Length];
        }

        #endregion


        #region private and internal methods
        //todo: add here anything that you need

        public static void DecomposeSwingTwist ( Quaternion q, Vector3 twistAxis, out Quaternion swing, out Quaternion twist)
        {
            Vector3 r = new Vector3(q.x, q.y, q.z);
            Vector3 p = Vector3.Project(r, twistAxis);
            twist = new Quaternion(p.x, p.y, p.z, q.w).normalized;
            swing = q * Quaternion.Inverse(twist);
        }

        void UpdateTentacle(int i, Vector3 target)
        {
            for (int t = _tentacles[i].Bones.Length - 2; t >= 0; t--)
            {
                Transform currentBone = _tentacles[i].Bones[t];
                Transform tipBone = _tentacles[i].Bones[_tentacles[i].Bones.Length - 1];

                Vector3 currentToTip = (tipBone.position - currentBone.position).normalized;
                Vector3 currentToTarget = (target - currentBone.position).normalized;

                Vector3 axis = Vector3.Cross(currentToTip, currentToTarget);
                float angle = Mathf.Min(_swingMax * Mathf.Deg2Rad, Vector3.Angle(currentToTip, currentToTarget));
                Quaternion q = Quaternion.AngleAxis(angle, axis);

                Quaternion qTwist;
                Quaternion qSwing;
                DecomposeSwingTwist(q, currentToTip.normalized, out qSwing, out qTwist);

                Quaternion newRotation = qSwing * currentBone.transform.rotation;

                currentBone.transform.rotation = newRotation;

                Vector3 vr = currentBone.localRotation.eulerAngles;
                vr.y = Mathf.Min(_twistMax, vr.y);
                currentBone.localRotation = Quaternion.Euler(vr);
            }
        }

        void update_ccd()
        {

            for (int i = 0; i < _tentacles.Length; i++)
            {
                Vector3 target = _targetBall[i] ? _ball.position : _randomTargets[i].position + _targetOffsets[i];
                UpdateTentacle(i, target);
            }
        }

        #endregion
    }
}
