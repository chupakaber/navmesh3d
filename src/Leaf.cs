using UnityEngine;

namespace ChupakaberGames.NavMesh3D {

    public class Leaf {
        public int Index = -1;
        public Vector3 Center = Vector3.zero;
        public bool StaticBlocked = false;
        public bool DynamicBlocked = false;
        public Leaf[] Neighbors = new Leaf[26];

        public bool InNeighbors(Leaf leaf) {
            if (leaf == null) {
                return false;
            }
            for (var i = 0; i < 26; i++) {
                if (Neighbors[i] == leaf) {
                    return true;
                }
            }
            return false;
        }
    }

}