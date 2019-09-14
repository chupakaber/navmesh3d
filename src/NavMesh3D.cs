using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using UnityEngine.Serialization;

// TODO:
// Serialization / Deserialization
// Check working
// Colliders layer mask

namespace ChupakaberGames.NavMesh3D {
    
    public class NavMesh3D : MonoBehaviour {

        public bool Threaded {
            get {
                return _threaded;
            }
            set {
                if (value) {
                    EnableThreading();
                } else {
                    DisableThreading();
                }
            }
        }

        [FormerlySerializedAs("Threaded")]
        [SerializeField]
        private bool _threaded = false;

        private VoxelMesh _voxelMesh = null;

        private Thread _thread = null;
        private bool _threadFlag = false;    
        private bool _requestPending = false;
        private bool _requestComplete = false;
        private bool _requestResult = false;
        private Vector3 _requestOrigin;
        private Vector3 _requestDestination;
        private List<Vector3> _requestCorners;
        private int _requestCornersCount;

        void Awake() {
            if (_voxelMesh == null) {
                _voxelMesh = new VoxelMesh();
            }
            if (_threaded) {
                EnableThreading();
            }
        }

        void OnDestroy() {
            Dispose();
        }

        void Dispose() {
            if (_voxelMesh != null) {
                _voxelMesh = null;
            }
            DisableThreading();
        }

        void EnableThreading() {
            _threaded = true;
            if (_thread == null) {
                _thread = new Thread(() => {
                    _threadFlag = true;
                    while (_threadFlag) {
                        ThreadWork();
                        Thread.Sleep(0);
                    }
                });
                _thread.Start();
            }
        }

        void DisableThreading() {
            _threaded = false;
            if (_thread != null) {
                _threadFlag = false;
                _thread.Interrupt();
                _thread = null;
            }
        }

        void ThreadWork() {
            if (_requestPending && !_requestComplete) {
                _voxelMesh.GetPath(_requestOrigin, _requestDestination, _requestCorners, out _requestCornersCount);
                _requestComplete = true;
            }
        }

        /// <summary> 
        /// Create a NavMesh3D from current scene environment
        /// </summary>
        /// <param name="size">Three dimensional NavMesh3D size of space in unity units.</param>
        public IEnumerator Bake(Vector3 size, Action callback) {
            yield return Bake(size, 1.0f, callback);
        }

        /// <summary> 
        /// Create a NavMesh3D from current scene environment
        /// </summary>
        /// <param name="size">Three dimensional NavMesh3D size of space in unity units.</param>
        /// <param name="leafSize">Size of NavMesh3D minimum node in unity units.</param>
        public IEnumerator Bake(Vector3 size, float leafSize, Action callback) {
            yield return Bake(size, -size / 2.0f, leafSize, callback);
        }

        /// <summary> 
        /// Create a NavMesh3D from current scene environment
        /// </summary>
        /// <param name="size">Three dimensional NavMesh3D size of space in unity units.</param>
        /// <param name="origin">Origin - minor corner of NavMesh3D space in unity units. Default is -0.5 of size</param>
        /// <param name="leafSize">Size of NavMesh3D minimum node in unity units.</param>
        public IEnumerator Bake(Vector3 size, Vector3 origin, float leafSize, Action callback) {
            _voxelMesh.Bake(size, origin, leafSize);
            callback();
            yield return true;
        }

        /// <summary> 
        /// Find shortest path
        /// </summary>
        /// <param name="origin">Starter point in world space.</param>
        /// <param name="destination">End point in world space.</param>
        /// <param name="corners">Resulting list of path points.</param>
        /// <param name="cornersCount">Count of actual path points in "corners" list.</param>
        public IEnumerator GetPath(Vector3 origin, Vector3 destination, List<Vector3> corners, Action<List<Vector3>, int> callback) {
            if (_threaded) {
                while (_requestPending) {
                    yield return new WaitForEndOfFrame();
                }
                _requestOrigin = origin;
                _requestDestination = destination;
                _requestCorners = corners;
                _requestPending = true;
                _requestComplete = false;
                while (!_requestComplete) {
                    yield return new WaitForEndOfFrame();
                }
                callback(_requestCorners, _requestCornersCount);
                _requestPending = false;
            } else {
                _voxelMesh.GetPath(origin, destination, corners, out _requestCornersCount);
                callback(_requestCorners, _requestCornersCount);
            }
        }

    }

}