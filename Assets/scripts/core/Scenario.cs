using System;
using System.Collections.Generic;
using UnityEngine;

namespace RVO
{

    abstract class Scenario : MonoBehaviour
    {
        /* Store the goals of the agents. */
        internal IList<RVO.Vector2> goals;
        public RVOSimulator sim_;
        public IList<Transform> agents;
        public IList<Transform> agents_label;

        /** Random number generator. */
        internal System.Random random;

        /** Constructor - Will be used in all Child class **/
        public Scenario()
        {
            goals = new List<RVO.Vector2>();
            random = new System.Random(3);//This Constructor take a seed in order to block the creation of random number
            // Other wise, you can choose a mutating seed (timestamp for example) in order to break the scenario from
            // an execution to another
            //sim_ = new RVOSimulator(0.895f, 0.5f, 8, 0.25f, 0.25f, 0.85f, 0.5f, 0.5f);//originale
            //RVOSimulator(float timeStep, float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed)
            sim_ = new RVOSimulator(0.1f, 0.5f, 8, 0.25f, 0.25f, 0.85f, 0.0f);//cleo 0.033f
        }
        /** Used to create Agents/Obstacles and process them **/
        public abstract void setupScenario();

        /** Used to set the velocity of all Agents considering their velocity and goal **/
        public  void setPreferredVelocities()
        {
            /*
           * Set the preferred velocity to be a vector of unit magnitude
           * (speed) in the direction of the goal.
           */
            for (int i = 0; i < sim_.getNumAgents(); ++i)
            {
                RVO.Vector2 goalVector = sim_.getAgentGoal(i) - sim_.getAgentPosition(i);

                if (RVO.Vector2.absSq(goalVector) > 1.0f)
                {
                    goalVector = RVO.Vector2.normalize(goalVector);
                }

                sim_.setAgentPrefVelocity(i, goalVector);

                // Perturb a little to avoid deadlocks due to perfect symmetry. 
                float angle = (float)random.NextDouble() * 2.0f * (float)Math.PI;
                float dist = (float)random.NextDouble() * 0.0001f;
                Vector2 aleasSpeed = new RVO.Vector2((float)Math.Cos(angle), (float)Math.Sin(angle));

                //Debug.Log("originale speed" + sim_.getAgentPrefVelocity(i) +  "///finalSpeed" + (sim_.getAgentPrefVelocity(i) +
                    //dist * aleasSpeed) + "/// num agent" + i);
                sim_.setAgentPrefVelocity(i, sim_.getAgentPrefVelocity(i) +
                    dist * aleasSpeed);
            }
        }

        /** Return true if all agents have reached the goal setup in the scenario, false in any other case**/
        public  bool reachedGoal()
        {
            /* Check if all agents have reached their goals. */
            for (int i = 0; i < sim_.getNumAgents(); ++i)
            {
                RVO.Vector2 pos = sim_.getAgentPosition(i);
                RVO.Vector2 goal = sim_.getAgentGoal(i);
                // condition for stopping. Strange to see 400 as parameter. It seems that 20m is correct !!!
                //if (RVO.Vector2.absSq(sim_.getAgentPosition(i) - sim_.getAgentGoal(i)) > 400.0f)
                //if ((Math.Abs(pos.x() - goal.x()) < 1.0f) && (Math.Abs(pos.y() - goal.y()) < 1.0f))
                //Debug.Log("absSq xx :" + RVO.Vector2.absSq(sim_.getAgentPosition(i) - sim_.getAgentGoal(i)));
               
                if ((RVO.Vector2.absSq(sim_.getAgentPosition(i) - sim_.getAgentGoal(i)) > 4.0f)){
                    
                    return false;
                }
            }

            return true;
        }


        /** Return the number of agents in the simulation **/
        public int getNumAgents()
        {
            return sim_.getNumAgents();
        }
        /** Do a step of the simulator **/
        internal void doStep(Boolean looped)
        {
            sim_.DoStep(looped);
        }
        //Nobby
        /////////////////
        /*public bool getIsArrived(int agentNo)
        {
            return sim_.agents_[agentNo].isArrived;
        }
        public void setIsArrived(int agentNo, bool isArrived)
        {
            sim_.agents_[agentNo].isArrived = isArrived;
        }*/
        ///////////////////
        /** Return the position of the Agents[i] in order to update his position on the scene**/
        public RVO.Vector2 getPosition(int i)
        {
            return sim_.getAgentPosition(i);
        }
        public RVO.Vector2 getAgentGoal(int i)
        {
            return sim_.getAgentGoal(i);
        }
        /** Change the position of the agent[i] in the simulation - Used to create corridor like scenario **/
        internal void setPosition(int i, RVO.Vector2 vector2)
        {
            sim_.agents_[i].position_ = vector2;
        }
        /** <abstract> Allow to instantiate a UnityObject into the current scene
         * with the shape of the transform given in parameters at the position given
         * <paramref name="position"/> Position where the agent should spawn
         * <paramref name="prefab"/> Shape of the agent that should be spawned
         */

        /****************************************/

        public void addAgent(Transform prefab, Vector3 position)
        {
            agents.Add(Instantiate(prefab, position, Quaternion.identity));
        }

        /** <abstract> Allow to instantiate a UnityObject into the current scene
        * with the shape of the transform given in parameters at the position given
        * with a specific radius
        * <paramref name="position"/> Position where the agent should spawn
        * <paramref name="prefab"/> Shape of the agent that should be spawned
        * <paramref name="radius_"/> Radius which should be givean to the agent
        */


        public void addAgent(Transform prefab, Vector3 position, float radius_)
        {
            agents.Add(Instantiate(prefab, position, Quaternion.identity));
            //agents[agents.Count - 1].transform.localScale = new Vector3(radius_, radius_, radius_);
        }
        /****************************************/

        public void addAgent(Transform prefab)
        {
            agents.Add(Instantiate(prefab));
        }


        public void addAgent(Transform prefab, Vector3 position, float radius_, int pos)
        {
            agents[pos] = (Instantiate(prefab, position, Quaternion.identity));
            //agents[pos].transform.localScale = new Vector3(radius_, radius_, radius_);
        }

    }
}