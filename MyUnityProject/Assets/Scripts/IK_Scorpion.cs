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
    public Transform _pointToLookAt;
    public float speedToRotate;
    private Vector3 _startPosition;
    private Vector3 _normalRotationVector;

    [Header("Tail")]
    public Transform tailTarget;
    public Transform tail;

    [Header("Legs")]
    public Transform[] legs;
    public Transform[] legTargets;
    public Transform[] legFutureBasesRayCasts;

    private bool active = true;

    // Start is called before the first frame update
    void Start()
    {
        _myController.InitLegs(legs,legFutureBasesRayCasts,legTargets);
        _myController.InitTail(tail);
        _startPosition = transform.position;
        _normalRotationVector = Body.position + Vector3.up;
        _pointToLookAt.position = _normalRotationVector; 
    }

    // Update is called once per frame
    void Update()
    {

        if (!active)
        {
            return;
        }

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

        if (!animPlaying)
        {
            return;
        }

        _normalRotationVector = _myController.GetMedianNormalTerrain();

        Debug.Log(Body.position + _normalRotationVector);
        Debug.Log(_pointToLookAt.position);

        Debug.DrawLine(Body.position, Body.position + _normalRotationVector, Color.red, 100);
        Debug.DrawLine(Body.position, _pointToLookAt.position, Color.blue, 100);

        _pointToLookAt.position = Vector3.Lerp(_pointToLookAt.position, _normalRotationVector + Body.position, speedToRotate * Time.deltaTime);

        Vector3 vectorToPointToLookAt = (_pointToLookAt.position - Body.position).normalized;

        float cosine = Vector3.Dot(Body.up, vectorToPointToLookAt);
        float angle = _myController.Rad2Deg(cosine);
        Vector3 crossVector = Vector3.Cross(Body.up, vectorToPointToLookAt);
        Body.rotation = Quaternion.AngleAxis(angle, crossVector) * Body.rotation;

        active = false;
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
