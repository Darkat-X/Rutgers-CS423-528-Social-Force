using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System;
using UnityEngine;
using UnityEngine.AI;

public class Agent : MonoBehaviour
{
    public float radius;
    public float mass;
    public float perceptionRadius;
    public const float k = 1.2f * 100000f;
    /* Mode
    1 = Pursue and Evade
    2 = Growing Spiral
    3 = Leader Following
    4 = Crowd Following
    other number = Normal Mode for Part one */
    private int Mode;
    private List<Vector3> path;
    private NavMeshAgent nma;
    private Rigidbody rb;
    private int count = 0;

    private HashSet<GameObject> perceivedNeighbors = new HashSet<GameObject>();
    private HashSet<GameObject> adjacentWalls = new HashSet<GameObject>();

    void Start()
    {
        //set the mode
        Mode = 3;
        path = new List<Vector3>();
        nma = GetComponent<NavMeshAgent>();
        rb = GetComponent<Rigidbody>();

        gameObject.transform.localScale = new Vector3(2 * radius, 1, 2 * radius);
        nma.radius = radius;
        rb.mass = mass;
        GetComponent<SphereCollider>().radius = perceptionRadius / 2;
    }

    private void Update()
    {
        count++;
        switch (Mode)
        {
            //Pursue and Evade
            case 1:
                Debug.Log("Mode 1 *");
                if (Char.GetNumericValue(name, 6) < 5)
                {
                    gameObject.tag = "Player";
                    var temp = gameObject.GetComponent<Renderer>();
                    temp.material.SetColor("_Color", Color.red);
                }
                else
                {
                    gameObject.tag = "Respawn";
                }
                break;
            //Growing Spiral
            case 2:
                Debug.Log("Mode 2 *");
                break;
            //Leader Following
            case 3:
                Debug.Log("Mode 3 *");
                if (Char.GetNumericValue(name, 6) < 1)
                {
                    gameObject.tag = "Player";
                    var temp = gameObject.GetComponent<Renderer>();
                    temp.material.SetColor("_Color", Color.yellow);
                }
                break;
            //Crowd Following
            case 4:
                Debug.Log("Mode 4 *");
                break;
            //Part one
            default:
                Debug.Log("Mode 5 *");
                break;
        }
        Debug.Log("The mode is : " + Mode.ToString());




        if (path.Count > 1 && Vector3.Distance(transform.position, path[0]) < 1.1f)
        {
            path.RemoveAt(0);
        } else if (path.Count == 1 && Vector3.Distance(transform.position, path[0]) < 2f)
        {
            path.RemoveAt(0);

            if (path.Count == 0)
            {
                //gameObject.SetActive(false);
                //AgentManager.RemoveAgent(gameObject);
            }
        }

        #region Visualization

        if (false)
        {
            if (path.Count > 0)
            {
                Debug.DrawLine(transform.position, path[0], Color.green);
            }
            for (int i = 0; i < path.Count - 1; i++)
            {
                Debug.DrawLine(path[i], path[i + 1], Color.yellow);
            }
        }

        if (false)
        {
            foreach (var neighbor in perceivedNeighbors)
            {
                Debug.DrawLine(transform.position, neighbor.transform.position, Color.yellow);
            }
        }

        #endregion
    }

    #region Public Functions

    public void ComputePath(Vector3 destination)
    {
        nma.enabled = true;
        var nmPath = new NavMeshPath();
        nma.CalculatePath(destination, nmPath);
        path = nmPath.corners.Skip(1).ToList();
        //path = new List<Vector3>() { destination };
        //nma.SetDestination(destination);
        nma.enabled = false;
    }

    public Vector3 GetVelocity()
    {
        return rb.velocity;
    }

    #endregion

    #region Incomplete Functions

    private Vector3 ComputeForce()
    {
        //var force = Vector3.zero;
        //var force = CalculateGoalForce(maxSpeed: 5) + CalculateAgentForce() + CalculateWallForce();

        //var force = CalculateSpiralForce(spinSpeed: 5, timeToGrowth: 5);

        //var force = CalculatePursuerForce();

        var force = Vector3.zero;
        Debug.Log("The mode is : " + Mode.ToString());
        switch (Mode)
        {
            //Pursue and Evade
            case 1:
                Debug.Log("Mode 1");
                force = CalculatePursuerForce();
                break;
            //Growing Spiral
            case 2:
                Debug.Log("Mode 2");
                force = CalculateSpiralForce(spinSpeed: 5, timeToGrowth: 2);
                break;
            //Leader Following
            case 3:
                Debug.Log("Mode 3");
                force = CalculateLeaderForce();
                break;
            //Crowd Following
            case 4:
                Debug.Log("Mode 4");
                force = CalculateCrowdForce();
                break;
            //Part one
            default:
            Debug.Log("Part one");
                force = CalculateGoalForce(maxSpeed: 5) + CalculateAgentForce() + CalculateWallForce();
                break;
        }

        if (force != Vector3.zero)
        {
            return force.normalized * Mathf.Min(force.magnitude, Parameters.maxSpeed);
        } else
        {
            return Vector3.zero;
        }
    }
    
    private Vector3 CalculateGoalForce(float maxSpeed)
    {
        if(path.Count == 0)
        {
            return Vector3.zero;
        }

        var temp = path[0] - transform.position;
        var desiredVel = temp.normalized * Mathf.Min(temp.magnitude, maxSpeed);
        var actualVelocity = rb.velocity;
        return mass * (desiredVel - actualVelocity) / Parameters.T;
    }

    private Vector3 CalculateAgentForce()
    {
        var agentForce = Vector3.zero;
        if (count < 2)
        {
            return agentForce;
        }
        foreach (var n in perceivedNeighbors) 
        {
            if (!AgentManager.IsAgent(n))
            {
                continue;
            }

            var neighbor = AgentManager.agentsObjs[n];
            var dir = (transform.position - neighbor.transform.position).normalized;
            var tangent = Vector3.Cross(Vector3.up, dir);
            var overlap = (radius + neighbor.radius) - Vector3.Distance(transform.position, n.transform.position);

            agentForce += (Parameters.A * Mathf.Exp(overlap / Parameters.B)) * dir;
            agentForce += Parameters.k * Mathf.Max(overlap, 0) * dir;
            agentForce -= Parameters.Kappa * (overlap > 0f ? overlap : 0) * Vector3.Dot(rb.velocity - neighbor.GetVelocity(), tangent) * tangent;
        }

        return agentForce;
    }

    private Vector3 CalculateWallForce()
    {
        var wallForce = Vector3.zero;
        if (count < 2)
        {
            return wallForce;
        }
        foreach (var n in adjacentWalls)
        {
            if (!WallManager.IsWall(n))
            {
                continue;
            }

            var dir = (transform.position - n.transform.position).normalized;
            var overlap = (radius + 0.5f) - Vector3.Distance(transform.position, n.transform.position);
            var tangent = Vector3.Cross(Vector3.up, dir);
            wallForce += Parameters.A * Mathf.Exp(overlap / Parameters.B) * dir;
            wallForce += Parameters.k * Mathf.Max(overlap, 0) * dir;
            wallForce -= Parameters.Kappa * (overlap > 0f ? overlap : 0) * Vector3.Dot(rb.velocity, tangent) * tangent;

        }

        return wallForce;
    }

    private Vector3 CalculatePursuerForce()
    {
        var pursuerForce = Vector3.zero;
        if (this.tag == "Player")
        {
            var dir = (closestTag("Respawn").transform.position - transform.position).normalized;
            var desiredVel = dir.normalized * Mathf.Min(dir.magnitude, 5);
            var actualVelocity = rb.velocity;
            var force = (desiredVel - actualVelocity) / Parameters.T;
            pursuerForce = force * 2 + CalculateAgentForce()/5 + CalculateWallForce();
            if (pursuerForce != Vector3.zero)
            {
                return pursuerForce;
            }
            else
            {
                return Vector3.zero;
            }
        }
        else
        {
            var dir = (transform.position - closestTag("Player").transform.position).normalized;
            var desiredVel = dir.normalized * Mathf.Min(dir.magnitude, 5);
            var actualVelocity = rb.velocity;
            var force = (desiredVel - actualVelocity) / Parameters.T;
            pursuerForce = force * 2 + CalculateAgentForce()/5 + CalculateWallForce();

            //Avoid Corner
            var wallForce = Vector3.zero;
            foreach (var n in adjacentWalls)
            {
                if (!WallManager.IsWall(n))
                {
                    continue;
                }
                if ((transform.position - n.transform.position).magnitude > 2f)
                {
                    continue;
                }
                
                var tempW = Vector3.zero - transform.position;
                var tangentW = Vector3.Cross(Vector3.up, tempW);
                var desiredVelW = 5 * tangentW.normalized * Mathf.Min(tangentW.magnitude, 5);
                var actualVelocityW = rb.velocity;
                wallForce = mass * (desiredVelW - 10 * actualVelocityW) / Parameters.T + CalculateGoalForce(maxSpeed: 5);
            }



            if (pursuerForce != Vector3.zero)
            {
                return pursuerForce + wallForce;
            }
            else
            {
                return Vector3.zero;
            }


        }
    }


    private Vector3 CalculateSpiralForce(float spinSpeed, float timeToGrowth)
    {
        var spiralForce = Vector3.zero;
        var otherForce = CalculateAgentForce() + CalculateWallForce();
        var goalForce = CalculateGoalForce(maxSpeed: 5);
        if (path.Count == 0)
        {
            return spiralForce;
        }
        var temp = path[0] - transform.position;
        var tangent = Vector3.Cross(Vector3.up, temp);
        var desiredVel = spinSpeed * tangent.normalized * Mathf.Min(tangent.magnitude, spinSpeed);
        var actualVelocity = rb.velocity;
        spiralForce += mass * (desiredVel - (2 * spinSpeed) * actualVelocity) / Parameters.T;
        if(count < timeToGrowth * 100) {
            spiralForce += otherForce + goalForce;
        }
        else
        {
            spiralForce += otherForce - goalForce;
        }
        return spiralForce;
    }

    //Leader Following Mode
    private Vector3 CalculateLeaderForce()
    {
        if (this.tag == "Player")
        {
            var leadForce = CalculateGoalForce(maxSpeed: 3) + CalculateAgentForce() + CalculateWallForce();
            var force = leadForce.normalized * Mathf.Min(leadForce.magnitude, Parameters.maxSpeed);
            if (force != Vector3.zero)
            {
                return force;
            }
            else
            {
                return Vector3.zero;
            }
        }
        else
        {
            var goalDirection = (closestTag("Player").transform.position - transform.position).normalized;
            var prefForce = (((goalDirection * Mathf.Min(goalDirection.magnitude, 1)) - rb.velocity) / Parameters.T);
            var leadForce = prefForce + CalculateAgentForce() + CalculateWallForce();
            var force = leadForce.normalized * Mathf.Min(leadForce.magnitude, Parameters.maxSpeed);

            //
            if (path.Count != 0)
            {
                if((path[0] - closestTag("Player").transform.position).magnitude > (path[0] - transform.position).magnitude)
                {
                    var tempL = Vector3.zero;
                    if ((path[0] - closestTag("Player").transform.position).x > (transform.position - closestTag("Player").transform.position).x)
                    {
                        tempL = closestTag("Player").transform.position - transform.position;
                    }
                    else
                    {
                        tempL = -closestTag("Player").transform.position - transform.position;
                    }
                    //var tempL = (closestTag("Player").transform.position - path[0]) - transform.position;


                    var tangentL = Vector3.Cross(Vector3.up, tempL);
                    var desiredVelL = 5 * tangentL.normalized * Mathf.Min(tangentL.magnitude, 5);
                    var actualVelocityL = rb.velocity;
                    force += mass * (desiredVelL - 10 * actualVelocityL) / Parameters.T + CalculateGoalForce(maxSpeed: 5);
                }
            }
            if (force != Vector3.zero)
            {
                return force;
            }
            else
            {
                return Vector3.zero;
            }
        }
    }

    //Crowd Following Mode
    private Vector3 CalculateCrowdForce()
    {
        var panicParameter = 0.7f;
        var goalDirection = ((1 - panicParameter) * (path[0] - transform.position));

        var neighborvel = Vector3.zero;
        foreach (var n in perceivedNeighbors)
        {
            neighborvel += ((path[0] - transform.position) * Mathf.Min((path[0] - transform.position).magnitude, 1));
        }

        neighborvel = neighborvel / perceivedNeighbors.Count;
        goalDirection = (goalDirection + panicParameter * neighborvel).normalized;

        var prefForce = (((goalDirection * Mathf.Min(goalDirection.magnitude, 1)) - rb.velocity) / Parameters.T);
        var force = prefForce + CalculateAgentForce() + CalculateWallForce();

        if (force != Vector3.zero)
        {
            return force.normalized * Mathf.Min(force.magnitude, Parameters.maxSpeed);
        }
        else
        {
            return Vector3.zero;
        }
    }



    public void ApplyForce()
    {
        var force = ComputeForce();
        force.y = 0;

        //rb.AddForce(force * 10, ForceMode.Force);
        rb.AddForce(force / mass, ForceMode.Acceleration);
    }

    public GameObject closestTag(string tag)
    {
        GameObject[] neighbor;
        neighbor = GameObject.FindGameObjectsWithTag(tag);
        GameObject closest = null;
        float distance = Mathf.Infinity;
        Vector3 position = transform.position;
        foreach (GameObject n in neighbor)
        {
            Vector3 diff = n.transform.position - position;
            float dis = diff.sqrMagnitude;
            if (dis < distance)
            {
                closest = n;
                distance = dis;
            }
        }
        return closest;
    }

    public void OnTriggerEnter(Collider other)
    {
        if (AgentManager.IsAgent(other.gameObject))
        {
            perceivedNeighbors.Add(other.gameObject);
        }
        if (WallManager.IsWall(other.gameObject))
        {
            adjacentWalls.Add(other.gameObject);
        }
    }
    
    public void OnTriggerExit(Collider other)
    {
        if (perceivedNeighbors.Contains(other.gameObject))
        {
            perceivedNeighbors.Remove(other.gameObject);
        }
        if (adjacentWalls.Contains(other.gameObject))
        {
            adjacentWalls.Remove(other.gameObject);
        }
    }

    public void OnCollisionEnter(Collision collision)
    {
        
    }

    public void OnCollisionExit(Collision collision)
    {
        
    }

    #endregion
}
