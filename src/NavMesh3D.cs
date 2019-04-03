using System.Collections.Generic;
using UnityEngine;

namespace ChupakaberGames.NavMesh3D {
    
    public class NavMesh3D : MonoBehaviour {

        public QuadTree quadTree = null;

        void Start() {
            quadTree = new QuadTree();
        }

        /// <summary> 
        /// Create a NavMesh3D from current scene environment
        /// </summary>
        /// <param name="size">Three dimensional NavMesh3D size of space in unity units.</param>
        public void Bake(Vector3 size) {
            Bake(size, 1.0f);
        }

        /// <summary> 
        /// Create a NavMesh3D from current scene environment
        /// </summary>
        /// <param name="size">Three dimensional NavMesh3D size of space in unity units.</param>
        /// <param name="leafSize">Size of NavMesh3D minimum node in unity units.</param>
        public void Bake(Vector3 size, float leafSize) {
            Bake(size, -size / 2.0f, leafSize);
        }

        /// <summary> 
        /// Create a NavMesh3D from current scene environment
        /// </summary>
        /// <param name="size">Three dimensional NavMesh3D size of space in unity units.</param>
        /// <param name="origin">Origin - minor corner of NavMesh3D space in unity units. Default is -0.5 of size</param>
        /// <param name="leafSize">Size of NavMesh3D minimum node in unity units.</param>
        public void Bake(Vector3 size, Vector3 origin, float leafSize) {
            quadTree.Bake(size, origin, leafSize);
        }

        /// <summary> 
        /// Find shortest path
        /// </summary>
        /// <param name="origin">Starter point in world space.</param>
        /// <param name="destination">End point in world space.</param>
        /// <param name="corners">Resulting list of path points.</param>
        /// <param name="cornersCount">Count of actual path points in "corners" list.</param>
        public bool GetPath(Vector3 origin, Vector3 destination, List<Vector3> corners, out int cornersCount) {
            return quadTree.GetPath(origin, destination, corners, out cornersCount);
        }

    }

}