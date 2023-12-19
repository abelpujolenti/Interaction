using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using OctopusController;
using UnityEngine.Serialization;

public class IK_Scorpion : MonoBehaviour
{
    private const int LAYER_TERRAIN = 6;
    private const int LAYER_MASK_TERRAIN = 1 << LAYER_TERRAIN;
    
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
    public float speedToElevate;
    private Vector3 _startPosition;
    private Vector3 _normalRotationVector;
    private Vector3 _bodyPosition;
    private float _distanceToFloor;

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
        _bodyPosition = Body.position;
        
        RaycastHit hit = RaycastToTerrain();

        _distanceToFloor = (hit.point - _bodyPosition).magnitude;
    }

    private RaycastHit RaycastToTerrain()
    {
        RaycastHit hit;
        Physics.Raycast(_bodyPosition, -Body.up, out hit, 1000, LAYER_MASK_TERRAIN);

        return hit;
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
        
        UpdateBodyPosition();
        
        if (animPlaying)
        {
            UpdateBodyPosition();
            if (Vector3.Distance(Body.position, EndPos.position) < 0.1f)
            {
                Body.position = EndPos.position;
                animPlaying = false;
            }
        }

        _myController.UpdateIK();

        if (!animPlaying)
        {
            return;
        }
        
        UpdateBodyRotation();
    }

    private void UpdateBodyPosition()
    {
        //_bodyPosition = Vector3.Lerp(_startPosition, EndPos.position, animTime / animDuration);
        
        RaycastHit hit = RaycastToTerrain();

        Vector3 vectorFromTerrainToBodyNormalized = (Body.position - hit.point).normalized;

        _bodyPosition = hit.point + vectorFromTerrainToBodyNormalized * _distanceToFloor;
        _bodyPosition.z = Mathf.Lerp(_startPosition.z, EndPos.position.z, animTime / animDuration);

        Body.position = Vector3.Lerp(Body.position, _bodyPosition, speedToElevate * Time.deltaTime);
    }

    private void UpdateBodyRotation()
    {
        _normalRotationVector = _myController.GetMedianNormalTerrain();

        _pointToLookAt.position = Vector3.Lerp(_pointToLookAt.position, _normalRotationVector + Body.position, speedToRotate * Time.deltaTime);
        
        Vector3 vectorToPointToLookAt = (_pointToLookAt.position - Body.position).normalized;
        float cosine = Vector3.Dot(Body.up, vectorToPointToLookAt);
        float angle = _myController.Rad2Deg(Mathf.Acos(cosine));
        Vector3 crossVector = Vector3.Cross(Body.up, vectorToPointToLookAt);
        Body.rotation = Quaternion.AngleAxis(angle, crossVector) * Body.rotation;
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
