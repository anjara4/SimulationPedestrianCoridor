using CenterSpace.Free;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Troschuetz.Random;
using UnityEngine.UI;
using System.Threading;
using System.IO;
using System.Linq;
using UnityEngine.SceneManagement;

namespace RVO
{
    class Samuel : Scenario
    {
        public class Pedestrian 
        {
            public int ID { get; set; }
            public float time { get; set; }
            public Vector2 pos { get; set; }
            public Vector2 veloc { get; set; }
            public float velocMed { get; set; }
            public int nb_neigb { get; set; }
            public int ID_leader { get; set; }
            public Vector2 POS_leader { get; set; }
            public Vector2 VELOC_leader { get; set; }
            public int nbTour { get; set; }
            public bool isAccFollow { get; set; }
            public bool isAccFollowGreter { get; set; }

            public Pedestrian(int ID, float time, Vector2 pos, Vector2 veloc, float velocMed, int nb_neigb,
                int ID_leader, Vector2 POS_leader, Vector2 VELOC_leader, int nbTour, bool isAccFollow, bool isAccFollowGreter)
            {
                this.ID = ID;
                this.time = time;
                this.pos = pos;
                this.veloc = veloc;
                this.velocMed = velocMed;
                this.nb_neigb = nb_neigb;
                this.ID_leader = ID_leader;
                this.POS_leader = POS_leader;
                this.VELOC_leader = VELOC_leader;
                this.nbTour = nbTour;
                this.isAccFollow = isAccFollow;
                this.isAccFollowGreter = isAccFollowGreter;
            }
        }
        private class Worker
        {
            private ManualResetEvent doneEvent_;
            internal IList<Agent> agents;
            public Vector2 vit_moy;
            private RVOSimulator simulator_;
            private bool looped;

            /**
             * <summary>Constructs and initializes a worker.</summary>
             *
             * <param name="start">Start.</param>
             * <param name="end">End.</param>
             * <param name="doneEvent">Done event.</param>
             */
            internal Worker(ManualResetEvent doneEvent, RVOSimulator simulator, bool looped)
            {
                agents = new List<Agent>();
                vit_moy = new Vector2();
                doneEvent_ = doneEvent;
                simulator_ = simulator;
                this.looped = looped;
            }

            /**
             * <summary>Performs a simulation step.</summary>
             *
             * <param name="obj">Unused.</param>
             */
            internal void addAgent(Agent a)
            {
                agents.Add(a);
            }

            internal void computeMedVelocity(object obj)
            {
                foreach (Agent a in agents)
                {
                    vit_moy += a.velocity_;
                    vit_moy /= 2;
                }
                doneEvent_.Set();
            }

            internal void clear(object obj)
            {
                agents.Clear();
                doneEvent_.Set();
            }
        }

        public Transform prefab;
        //public Transform human;

        public Samuel() : base() { }
        int ped_num_;
        //int model_type_ = 0;
        
        bool group_ = false;
        int fluxes_ = 1;
        float corridor_angle_ = 0;
        bool loop_ = true;
        List<Color> colors = new List<Color>();
        List<int> step_stop = new List<int>();
        //public Toggle follow_but_;
        //public Toggle save_but_;
        //public Toggle hum_but_;
        //public Slider camera_height_;
        private IList<Worker> workers;
        private ManualResetEvent[] doneEvents_;
        //public Boolean human_prefab = false;
        //public new string name;

        List<Pedestrian> list_peds_nic = new List<Pedestrian>();
        bool is_saved;// = false;
        float ped_radius_;// = 0.3f;
        float time_step;// = 0.1f;
        float neighborDist;// = 10;
        int maxNeighbors;// = 10;
        float timeHorizon;// = 1;
        float timeHorizonObst;// = 1;
        float corridor_length_;// = 30;
        float corridor_width_;// = 10;
        bool follow_;// = true;
        static int nb_ped;// = 400;
        int time_stop;// = 300;
        static int num_type_follow;// = 11;
        

        void saveData01(List<Pedestrian> list_ped)
        {
            string path_keep_data = "Data/COSYS_CROWD/";
            string details_ped = nb_ped.ToString() + "_Case_" + num_type_follow.ToString() + "Test.csv";
            System.IO.Directory.CreateDirectory(path_keep_data);
            File.Create(path_keep_data + details_ped).Dispose();
            using (TextWriter tw = new StreamWriter(path_keep_data + details_ped))
                tw.WriteLine("Num" + '\t' + "Time" + '\t' + "X" + '\t' + "Y" + '\t' + "Vx" + '\t' + "Vy" + '\t' + "V" + '\t' + "NbNeigb" + '\t' + "IdLeader" + '\t' + 
                    "PosLeadeX" + '\t' + "PosLeadeY" + '\t' + "VeloLeaderX" + '\t' + "VeloLeaderY" + '\t' + "DistToLead" + '\t' + "NbTour" + '\t' + "isAccFollow" + '\t' + "isAccFollowGreateer");
            double distTolead = -1;
            foreach (Pedestrian peds in list_ped)
            {
                if (peds.ID_leader != -1)
                    distTolead = Math.Sqrt((peds.POS_leader.x_ - peds.pos.x_) * (peds.POS_leader.x_ - peds.pos.x_) + (peds.POS_leader.y_ - peds.pos.y_) * (peds.POS_leader.y_ - peds.pos.y_));
                else distTolead = -1;
                using (TextWriter tw = new StreamWriter(path_keep_data + details_ped, true))
                    tw.WriteLine(peds.ID + "\t" + Math.Round(peds.time, 1) + "\t" + peds.pos.x_ + "\t" + peds.pos.y_ + "\t" +
                        peds.veloc.x_ + "\t" + peds.veloc.y_ + "\t" + peds.velocMed + "\t" + peds.nb_neigb + "\t" + peds.ID_leader + "\t" + peds.POS_leader.x_ + "\t" +
                        peds.POS_leader.y_ + "\t" + peds.VELOC_leader.x_ + "\t" + peds.VELOC_leader.y_ + "\t" + distTolead + "\t" + peds.nbTour + "\t" +
                        peds.isAccFollow + "\t" + peds.isAccFollowGreter);
            }
        }


        void Start()
        {
            is_saved = MainProject.get_is_saved();
            ped_radius_ = MainProject.get_ped_radius_();
            time_step = MainProject.get_time_step();
            neighborDist = MainProject.get_neighborDist();
            maxNeighbors = MainProject.get_maxNeighbors();
            timeHorizon = MainProject.get_timeHorizon();
            timeHorizonObst = MainProject.get_timeHorizonObst();
            corridor_length_ = MainProject.get_corridor_length_();
            corridor_width_ = MainProject.get_corridor_width_();
            follow_ = MainProject.get_follow_();
            nb_ped = MainProject.get_nb_ped();
            time_stop = MainProject.get_time_stop();
            num_type_follow = MainProject.get_num_type_follow();
            
            //camera_height_.minValue = 10;
            //camera_height_.maxValue = 80;
            //camera_height_.value = Camera.main.transform.position.y;
            //Camera.main.transform.position = new Vector3(corridor_length_ / 2, Camera.main.transform.position.y, Camera.main.transform.position.z);

            colors.Add(new Color(0.647f, 0.165f, 0.165f)); //Col_Brown
            colors.Add(new Color(0.000f, 1.000f, 1.000f)); //Col_Cyan
            colors.Add(new Color(0.000f, 0.392f, 0.000f)); //Col_DarkGreen
            colors.Add(new Color(0.282f, 0.239f, 0.545f)); //Col_DarkSlateBlue
            colors.Add(new Color(0.604f, 0.804f, 0.196f));
            colors.Add(new Color(0.804f, 0.804f, 0.196f));
            colors.Add(new Color(0.804f, 0.804f, 0.096f));
            colors.Add(new Color(0.94f, 0.804f, 0.10f));

            agents = new List<Transform>();
            Application.targetFrameRate = 60;
            Application.runInBackground = true;
            sim_.setAgentDefaults(1f, 10, 1, 1, ped_radius_, 2, new Vector2(0, 0), 0.5f);
            //ped_num_ = (int)(1.9f * corridor_width_ * (corridor_length_ + 10));
            ped_num_ = nb_ped;
            sim_.setTimeStep(time_step);
            transform.localScale = new Vector3(corridor_length_ / 10, 1, corridor_width_ / 10);
            transform.position = new Vector3(corridor_length_ / 2, 0, corridor_width_ / 2);
            //my_corridor
            transform.GetComponent<MeshRenderer>().material.color = colors[0];
            setupScenario();
            //follow_but_.isOn = follow_;

            int worker_num = 4;// (int)((corridor_length_-20) / (8 * ped_radius_)) + 2;
            initializeWorker(worker_num);

            sim_.initialize_virtual_and_agents();
            sim_.processObstacles();

            sim_.kdTree_.buildAgentTree(true);
        }

        private void initializeWorker(int worker_num)
        {
            workers = new Worker[worker_num];
            doneEvents_ = new ManualResetEvent[workers.Count];

            for (int block = 0; block < workers.Count; ++block)
            {
                doneEvents_[block] = new ManualResetEvent(false);
                workers[block] = new Worker(doneEvents_[block], sim_, true);
            }


            updateWorker(worker_num);
        }

        private void updateWorker(int worker_num)
        {
            foreach (Agent a in sim_.agents_)
            {
                if (a.position_.x_ <= corridor_length_ / 4)
                {
                    workers[0].addAgent(a);
                }

                else if (a.position_.x_ <= 2 * corridor_length_ / 4)
                {
                    workers[1].addAgent(a);
                }
                else if (a.position_.x_ <= 3 * corridor_length_ / 4)
                {
                    workers[2].addAgent(a);
                }
                else
                {
                    workers[3].addAgent(a);
                }
            }
        }
        
        // Update is called once per frame
        void Update()
        {
            for (int block = 0; block < workers.Count; ++block)
            {
                doneEvents_[block].Reset();
                ThreadPool.QueueUserWorkItem(workers[block].clear);
            }

            WaitHandle.WaitAll(doneEvents_);
            updateWorker(workers.Count);
            if (!reachedGoal())
            {
                /*if (hum_but_.isOn && !human_prefab)
                {
                    int range = agents.Count;
                    for (int i = 0; i < range; i++)
                    {
                        Destroy(agents[i].gameObject);
                        addAgent(human, agents[i].position, sim_.getDefaultRadius(), i);
                    }
                    human_prefab = true;

                }
                else if (!hum_but_.isOn && human_prefab)
                {
                    int range = agents.Count;
                    for (int i = 0; i < range; i++)
                    {
                        Destroy(agents[i].gameObject);
                        addAgent(prefab, agents[i].position, sim_.getDefaultRadius(), i);

                    }
                    human_prefab = false;
                }*/
                setAgentsProperties();
                setPreferredVelocities();
                sim_.initialize_virtual_and_agents();
                
                for (int i = 0; i < getNumAgents(); i++)
                {
                    Vector2 agent_position = sim_.getAgentPosition(i);
                    if (agent_position.x_ < 5)
                    {
                        Vector2 p1 = agent_position + new Vector2(corridor_length_, 0);
                        sim_.addVirtualAgent(0, p1);
                    }

                    
                    float velocityped = (float)Math.Sqrt(sim_.getAgentVelocity(i).x_ * sim_.getAgentVelocity(i).x_ + sim_.getAgentVelocity(i).y_ * sim_.getAgentVelocity(i).y_);
                    float veloX = sim_.getAgentVelocity(i).x_;
                    Pedestrian peds = new Pedestrian(i, sim_.globalTime_, sim_.getAgentPosition(i), sim_.getAgentVelocity(i), velocityped,
                        sim_.getAgentNumAgentNeighbors(i), sim_.getAgentLeaderNo(i), sim_.getAgentLeaderPosition(i), 
                        sim_.getAgentLeaderVelocity(i), sim_.getAgentNbTour(i), sim_.getAgentIsAccFollow(i), sim_.getAgentIsAccFollowGreater(i));
                    list_peds_nic.Add(peds);

                   
                    //if (sim_.globalTime_ >= time_stop && is_saved == false) //begin backup at the end of simulation time
                    //{
                        //Debug.Log("End simu - nb ped : " + nb_ped);
                        //saveData01(list_peds_nic);
                        //is_saved = true;
                        //SceneManager.LoadScene(1);
                        //MainProject.set_is_saved(true);
                        //Debug.Break();
                    //}
                }

                doStep(true);

                /* Output the current global time. */
                //print(Simulator.Instance.getGlobalTime());

                /*if (follow_but_.isOn != follow_)
                {
                    follow_ = follow_but_.isOn;

                    for (int i = 0; i < getNumAgents(); ++i)
                    {
                        sim_.agents_[i].follows_ = follow_;
                    }
                }*/
                /*if (save_but_.isOn != sim_.save)
                {
                    sim_.save = save_but_.isOn;

                }*/
                Vector3 pos3 = Camera.main.transform.position;
                //Camera.main.transform.position = new Vector3(pos3.x, camera_height_.value, pos3.z);
                Camera.main.transform.position = new Vector3(15, 17, 5);
                //Camera.main.transform.rotation = new Quaternion(90, 0, 0, 0);
                //Camera.main.transform.Rotate(90, 0, 0);
                Camera.main.transform.rotation = Quaternion.LookRotation(new Vector3(0, -90, 0));

                int totot = getNumAgents();
                for (int i = 0; i < getNumAgents(); ++i)
                {
                    Vector2 position = sim_.getAgentPosition(i);
                    agents[i].transform.position = new Vector3(position.x(), 0f, position.y());
                    RVO.Vector2 vector_cam = sim_.getAgentVelocity(i);
                    agents[i].rotation = Quaternion.LookRotation(new Vector3(vector_cam.x_, 0, vector_cam.y_));

                    //if (!human_prefab)
                        //setColorStopGo(i); //setColor(i);
                }
            }
            else
            {
                for (int i = 0; i < getNumAgents(); ++i)
                {
                    agents[i].transform.GetComponent<Rigidbody>().isKinematic = true;
                }
            }

            for (int block = 0; block < workers.Count; ++block)
            {
                doneEvents_[block].Reset();
                ThreadPool.QueueUserWorkItem(workers[block].computeMedVelocity);
            }

            WaitHandle.WaitAll(doneEvents_);
            
        }

        void setCorridor()
        {// Add (polygonal) obstacle(s), specifying vertices in counterclockwise order.
         // Add corridor right side

            IList<Vector2> right_side = new List<Vector2> {
            Vector2.rotation(new Vector2(corridor_length_ + 100, 0.0f), corridor_angle_),
            Vector2.rotation(new Vector2(-100.0f, 0.0f), corridor_angle_),
            Vector2.rotation(new Vector2(-100.0f, -50.0f), corridor_angle_),
            Vector2.rotation(new Vector2(corridor_length_ + 100, -50.0f), corridor_angle_)};
            sim_.addObstacle(right_side);
            //Add cooridor left side
            IList<Vector2> left_side = new List<Vector2> {
            Vector2.rotation(new Vector2(-100.0f, corridor_width_), corridor_angle_),
            Vector2.rotation(new Vector2(corridor_length_ + 100, corridor_width_), corridor_angle_),
            Vector2.rotation(new Vector2(corridor_length_ + 100, corridor_width_ + 50.0f), corridor_angle_),
            Vector2.rotation(new Vector2(-100.0f, corridor_width_ + 50.0f), corridor_angle_) };
            sim_.addObstacle(left_side);


            IList<Vector2> begin_side = new List<Vector2> {
             Vector2.rotation(new Vector2(-0, -50), corridor_angle_),
             Vector2.rotation(new Vector2(-0 , corridor_width_+50), corridor_angle_),
             Vector2.rotation(new Vector2(-10, corridor_width_ + 50.0f), corridor_angle_),
             Vector2.rotation(new Vector2(-10, 0), corridor_angle_) };
            sim_.addObstacle(begin_side);
            // Process obstacles so that they are accounted for in the simulation.
            sim_.processObstacles();
        }

        void placeAgents()
        {
            NormalDistribution normal = new NormalDistribution();
            normal.Mu = 1.4;
            normal.Sigma = Math.Sqrt(0.2);

            MT19937Generator generator = new MT19937Generator();
            StandardGenerator sg = new StandardGenerator();

            ContinuousUniformDistribution x_distribution = new ContinuousUniformDistribution(generator);
            NormalDistribution y_distribution = new NormalDistribution(generator);
            y_distribution.Mu = corridor_width_ / 2;
            y_distribution.Sigma = sim_.getDefaultRadius();

            for (int ped = 0; ped < ped_num_; ped++)
            {  // Place Agent
                float x = (float)x_distribution.NextDouble() * corridor_length_ % corridor_length_;
                float y = (float)((y_distribution.NextDouble() * corridor_width_) - 9) % corridor_width_;

                Vector2 position = new Vector2(x, Math.Abs(y));
                position = Vector2.rotation(position, corridor_angle_);
                //sim_.addAgent(position, model_type_, follow_, group_, false);
                //limit speed values
                float my_speed = (float)normal.NextDouble();
                if (my_speed > 2.0f) my_speed = 2.0f;
                if (my_speed < 0.3f) my_speed = 0.3f;
                
                //sim_.setAgentMaxSpeed(ped, 0.3f);
                                
                //sim_.addRVOAgent(position, follow_, group_, neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, ped_radius_, my_speed, new RVO.Vector2(0, 0), 0.5f);
                sim_.addRVOAgent(position, follow_, group_, 10, 10, 1, 1, 0.3f, my_speed, new RVO.Vector2(0, 0), 0.5f);

                sim_.setModel_follow_type(ped, num_type_follow); //type follow initialisation //same for all agent in our case

                addAgent(prefab, new Vector3(position.x(), 0, position.y()), sim_.getDefaultRadius());

                step_stop.Add(0);

                // Set agent's goal
                Vector2 corridor_end = new Vector2(corridor_length_ + 100, y);
                corridor_end = Vector2.rotation(corridor_end, corridor_angle_);
                sim_.setAgentGoal(ped, corridor_end);
            }
        }

        void setAgentsProperties()
        {
            for (int i = 0; i < sim_.getNumAgents(); ++i)
            {  // Set Agent Goal
                Vector2 pos = sim_.getAgentPosition(i);
                Vector2 goal = sim_.getAgentGoal(i);
                // Position in the corridor referential
                Vector2 local_pos = Vector2.rotation(pos, -corridor_angle_);
                Vector2 local_goal = Vector2.rotation(goal, -corridor_angle_);
                // Set agent goal
                Vector2 new_goal = new Vector2(local_goal.x(), local_pos.y());
                // Back to world's referential
                new_goal = Vector2.rotation(new_goal, corridor_angle_);
                // Set goal
                sim_.setAgentGoal(i, new_goal);
                
                // Set Agent Position (looped corridor)
                // If the agent as reached the end of the corridor (case 1)
                if (local_pos.x() >= corridor_length_ && local_goal.x() > corridor_length_)
                {
                    // Put at the start of the corridor
                    Vector2 new_pos = new Vector2(local_pos.x() - (corridor_length_), local_pos.y());
                    // Back to world's referential
                    new_pos = Vector2.rotation(new_pos, corridor_angle_);
                    // Add agent
                    sim_.setAgentPosition(i, new_pos);
                    // Save agent's data
                    //DataSaving::saveAgentData(sim_, i, follow_);
                    // Reinitialize data
                    //sim_.reinitializeOutputVariables(i);

                    //increase number of tour
                    int nbTour = sim_.getAgentNbTour(i);
                    sim_.setAgentNbTour(i, nbTour + 1);
                }
                if (pos.y() > corridor_width_ || pos.y() < 0)
                {
                    System.Random rand = new System.Random();
                    Vector2 new_pos = new Vector2(pos.x_, rand.Next((int)corridor_width_ + 1));
                    // Back to world's referential
                    new_pos = Vector2.rotation(new_pos, corridor_angle_);
                    // Add agent
                    sim_.setAgentPosition(i, new_pos);
                    // Save agent's data
                    //DataSaving::saveAgentData(sim_, i, follow_);
                    // Reinitialize data
                    //sim_.reinitializeOutputVariables(i);
                }

                // If the agent as reached the end of the corridor (case 2)
                if (local_pos.x() < 0 && local_goal.x() < 0)
                {
                    // Put at the start of the corridor
                    Vector2 new_pos = new Vector2(local_pos.x() + corridor_length_, local_pos.y());
                    // Back to world's referential
                    new_pos = Vector2.rotation(new_pos, corridor_angle_);
                    // Add agent
                    sim_.setAgentPosition(i, new_pos);
                    // Save agent's data
                    //DataSaving::saveAgentData(sim_, i, following_behavior_);
                    // Reinitialize data
                    //sim_.reinitializeOutputVariables(i);
                }
            }
        }

        private void setVelocity(int i, RVO.Vector2 vector2)
        {
            sim_.agents_[i].velocity_ = vector2;
        }

        public override void setupScenario()
        {
            setCorridor();
            placeAgents();
        }
    }
}
