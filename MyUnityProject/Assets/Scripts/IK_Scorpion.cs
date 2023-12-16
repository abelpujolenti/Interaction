using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using OctopusController;
using UnityEngine.Serialization;

public class IK_Scorpion : MonoBehaviour
{
    MyScorpionController _myController= new MyScorpionController();

    public IK_tentacles _myOctopus;

    [Header("Body")]
    float animTime;
    public float animDuration;
    bool animPlaying = false;
    public Transform Body;
    public Transform EndPos;
    private Vector3 _startPosition;

    [Header("Tail")]
    public Transform tailTarget;
    public Transform tail;

    [Header("Legs")]
    public Transform[] legs;
    public Transform[] legTargets;
    public Transform[] legFutureBasesRayCasts;
    public Transform[] legFutureBases;

    // Start is called before the first frame update
    void Start()
    {
        _myController.InitLegs(legs,legFutureBasesRayCasts,legTargets, legFutureBases);
        _myController.InitTail(tail);
        _startPosition = transform.position;
    }

    // Update is called once per frame
    void Update()
    {
        if(animPlaying)
            animTime += Time.deltaTime;

        NotifyTailTarget();
        
        if (Input.GetKeyDown(KeyCode.Space))
        {
            NotifyStartWalk();
            animTime = 0;
            animPlaying = true;
        }

        if (animTime < animDuration)
        {
            Body.position = Vector3.Lerp(_startPosition, EndPos.position, animTime / animDuration);
        }
        else if (animTime >= animDuration && animPlaying)
        {
            Body.position = EndPos.position;
            animPlaying = false;
        }

        _myController.UpdateIK();
    }
    
    //Function to send the tail target transform to the dll
    public void NotifyTailTarget()
    {
        _myController.NotifyTailTarget(tailTarget);
    }

    //Trigger Function to start the walk animation
    public void NotifyStartWalk()
    {

        _myController.NotifyStartWalk();
    }
}
