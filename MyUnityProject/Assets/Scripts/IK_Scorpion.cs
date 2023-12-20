using OctopusController;
using UnityEngine;
using UnityEngine.Serialization;

public class IK_Scorpion : MonoBehaviour
{
    private const int LAYER_TERRAIN = 6;
    private const int LAYER_MASK_TERRAIN = 1 << LAYER_TERRAIN;
    
    MyScorpionController _myController= new MyScorpionController();

    public IK_tentacles _myOctopus;

    [Header("Body")]
    bool animPlaying;
    public Transform Body;
    public Transform EndPos;
    public Transform _pointToLookAt;
    public float speedToRotate;
    public float speedToMove;
    private Vector3 _normalRotationVector;
    private Vector3 _desiredBodyPosition;
    public Transform _pointToFollow;
    private float _distanceToFloor;

    [Header("Tail")]
    public Transform tailTarget;
    public Transform tail;

    [Header("Legs")]
    public Transform[] legs;
    public Transform[] legTargets;
    public Transform[] legFutureBasesRayCasts;

    // Start is called before the first frame update
    void Start()
    {
        _myController.InitLegs(legs,legFutureBasesRayCasts,legTargets);
        _myController.InitTail(tail);
        _normalRotationVector = Body.position + Vector3.up;
        _pointToLookAt.position = _normalRotationVector;
        _pointToFollow.position = Body.position;
        
        RaycastHit hit = RaycastToTerrain();

        _distanceToFloor = (hit.point - _pointToFollow.position).magnitude;
        
        Debug.Log(_distanceToFloor);
    }

    private RaycastHit RaycastToTerrain()
    {
        RaycastHit hit;
        Physics.Raycast(Body.position, -Body.up, out hit, 1000, LAYER_MASK_TERRAIN);

        return hit;
    }

    // Update is called once per frame
    void Update()
    {
        NotifyTailTarget();
        
        if (Input.GetKeyDown(KeyCode.Space))
        {
            NotifyStartWalk();
            animPlaying = true;
        }
        
        if (animPlaying)
        {
            UpdateBodyPosition();
            UpdateBodyRotation();
            
            if (Vector3.Distance(Body.position, EndPos.position) < 0.1f)
            {
                Body.position = EndPos.position;
                animPlaying = false;
            }
        }

        _myController.UpdateIK();
    }

    private void UpdateBodyPosition()
    {
        RaycastHit hit = RaycastToTerrain();

        Vector3 vectorFromTerrainToBodyNormalized = (Body.position - hit.point).normalized;

        Vector3 bodyPosition = hit.point + vectorFromTerrainToBodyNormalized * _distanceToFloor;
        
        _desiredBodyPosition = _pointToFollow.position;
        _desiredBodyPosition.y = bodyPosition.y;
        
        Body.position = Vector3.Lerp(Body.position, _desiredBodyPosition, speedToMove * Time.deltaTime);
    }

    private void UpdateBodyRotation()
    {
        Vector3 vectorToFollow = (_desiredBodyPosition - Body.position).normalized;
        float cosine = Vector3.Dot(-Body.forward, vectorToFollow);
        float angle = _myController.Rad2Deg(Mathf.Acos(cosine));
        Vector3 crossVector = Vector3.Cross(-Body.forward, vectorToFollow);
        Body.rotation = Quaternion.AngleAxis(angle, crossVector) * Body.rotation;
        
        _normalRotationVector = _myController.GetMedianNormalTerrain();

        _pointToLookAt.position = Vector3.Lerp(_pointToLookAt.position, _normalRotationVector + Body.position, speedToRotate * Time.deltaTime);
        
        Vector3 vectorToPointToLookAt = (_pointToLookAt.position - Body.position).normalized;
        cosine = Vector3.Dot(Body.up, vectorToPointToLookAt);
        angle = _myController.Rad2Deg(Mathf.Acos(cosine));
        crossVector = Vector3.Cross(Body.up, vectorToPointToLookAt);
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
