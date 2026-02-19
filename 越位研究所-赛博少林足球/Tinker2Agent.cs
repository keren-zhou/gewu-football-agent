using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Policies;
using Random = UnityEngine.Random;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using Unity.Sentis;
using System.Threading.Tasks;


public class Tinker2Agent : Agent
{
    int tp = 0;
    int tt = 0;
    int tq = 0;      // 左脚踢球计时器
    int tr = 0;      // 右脚踢球计时器
    int tw = 0;      // 踢球后等待计时器

    public bool fixbody = false;
    public bool train;
    public bool accelerate;
    float uf1 = 0;
    float uf2 = 0;
    float[] u = new float[12] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    float[] utotal = new float[12] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    int T1 = 50;
    int T2 = 30;
    int tp0 = 0;
    Transform body;
    public int ObservationNum;
    public int ActionNum;

    List<float> P0 = new List<float>();
    List<float> W0 = new List<float>();
    List<Transform> bodypart = new List<Transform>();
    Vector3 pos0;
    Vector3 posball0;
    Quaternion rot0;
    ArticulationBody[] arts = new ArticulationBody[40];
    ArticulationBody[] acts = new ArticulationBody[20];
    GameObject robot;

    float[] kb = new float[12] { 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30 };
    float ko = 0.4f;
    public float vr = 0;
    public float wr = 0;
    public float cr = 0.5f;
    public bool disturb = false;
    public bool wasd = false;
    public bool keyboard = false;
    bool l_kick = false;
    bool r_kick = false;
    bool wait = false;
    public Transform ball;
    public Transform rival;
    public Transform goal;  // 球门位置（可选，如果为空则使用默认位置 x=15.8）

    // 模型切换相关变量
    public enum ModelType { Tinker2, Tinker3 }
    private ModelType currentModel = ModelType.Tinker2;
    private BehaviorParameters behaviorParameters;
    public ModelAsset tinker2Model;  // Tinker2追踪模型资源（在Inspector中分配）
    public ModelAsset tinker3Model;  // Tinker3起身模型资源（在Inspector中分配）
    public float fallAngleThreshold = 30f;  // 摔倒角度阈值
    public float standUpAngleThreshold = 15f;  // 起身成功角度阈值
    public int standUpStableFrames = 60;  // 需要保持稳定多少帧才算成功起身
    private int standUpStableCounter = 0;  // 当前稳定帧计数
    private bool isFallen = false;  // 是否处于摔倒状态

    // 训练模式相关变量
    public int trainingMoveMode = 0;      // 当前训练移动模式
    public int trainingMoveDuration = 0;      // 当前模式持续时间
    public int trainingMoveChangeInterval = 300;      // 每300帧切换一次移动模式
    public int trainingKickTriggerTimer = 0;      // 训练模式下踢球触发计时器
    public int trainingKickInterval = 300;      // 训练模式下每300帧可能触发一次踢球
    public int trainingForceTriggerTimer = 0;      // 训练模式下推力触发计时器
    public int trainingForceInterval = 100;      // 训练模式下每100帧可能触发一次推力
    public float trainingVr = 0f;      // 训练模式下的目标前进速度
    public float trainingWr = 0f;      // 训练模式下的目标角速度

    // 课程训练相关变量
    private int currentLesson = 0;      // 当前课程阶段（0: 静态平衡, 1: 动态平衡, 2: 动态+踢球, 3: 动态+踢球+推力）
    private float maxSpeedScale = 0f;   // 当前课程阶段的最大速度缩放（0-1）

    // Tinker机器人踢球关节索引（根据实际关节信息设置）
    // 右腿：R0_Link=1, R1_Link=2, R2_Link=3, R3_Link=4, R4_Link_ankle=5
    // 左腿：L0_Link=6, L1_Link=7, L2_Link=8, L3_Link=9, L4_Link_ankle=10
    int L0_idx = 6, L1_idx = 7, L2_idx = 8, L3_idx = 9;      // 左腿关节
    int R0_idx = 1, R1_idx = 2, R2_idx = 3, R3_idx = 4;      // 右腿关节

    bool IsValidJointIndex(int idx)
    {
        return idx >= 0 && idx < arts.Length && arts[idx] != null;
    }

    public override void Initialize()
    {
        arts = this.GetComponentsInChildren<ArticulationBody>();
        ActionNum = 0;
        for (int k = 0; k < arts.Length; k++)
        {
            if(arts[k].jointType.ToString() == "RevoluteJoint")
            {
                acts[ActionNum] = arts[k];
                print(acts[ActionNum]);
                ActionNum++;
            }
        }
        ActionNum = 10;
        body = arts[0].GetComponent<Transform>();
        pos0 = body.position;
        if(ball != null) posball0 = ball.position;
        rot0 = body.rotation;
        arts[0].GetJointPositions(P0);
        arts[0].GetJointVelocities(W0);
        accelerate = train;

        // 获取BehaviorParameters组件用于模型切换
        behaviorParameters = GetComponent<BehaviorParameters>();
        if (behaviorParameters == null)
        {
            behaviorParameters = gameObject.AddComponent<BehaviorParameters>();
        }

        // 初始化使用Tinker2模型
        currentModel = ModelType.Tinker2;
        isFallen = false;
        standUpStableCounter = 0;
        if (tinker2Model != null && !train)
        {
            behaviorParameters.Model = tinker2Model;
        }

        string printText = $"Total ArticulationBodies: {arts.Length}";
        for (int k = 0; k < arts.Length; k++)
        {
            if (arts[k].dofCount == 0) continue;
            // printText += $"L{k}: {arts[k].name} {arts[k].jointType.ToString()} \n";
            printText += $"L{k}: {arts[k].name} {arts[k].jointType} {(arts[k].jointPosition[0] * Mathf.Rad2Deg):F1}° \n";
        }
        print(printText);
        if (L0_idx >= arts.Length || L1_idx >= arts.Length || L2_idx >= arts.Length || L3_idx >= arts.Length ||
            R0_idx >= arts.Length || R1_idx >= arts.Length || R2_idx >= arts.Length || R3_idx >= arts.Length)
        {
            print("Warning: Some joint indices are out of range! Please adjust the indices.");
        }
    }

    private bool _isClone = false;
    void Start()
    {
        Time.fixedDeltaTime = 0.01f;

        int numrob=8;
        if(train)numrob=32;
        if(fixbody || keyboard)numrob=0;
        if (train && !_isClone)
        {
            for (int i = 1; i < numrob; i++)
            {
                GameObject clone = Instantiate(gameObject);
                //clone.transform.position = transform.position + new Vector3(i * 2f, 0, 0);
                clone.name = $"{name}_Clone_{i}";
                clone.GetComponent<Tinker2Agent>()._isClone = true;
            }
        }
    }
    void ChangeLayerRecursively(GameObject obj, int targetLayer)
    {
        obj.layer = targetLayer;
        foreach (Transform child in obj.transform)ChangeLayerRecursively(child.gameObject, targetLayer);
    }

    public override void OnEpisodeBegin()
    {
        tp = 0;
        tt = 0;
        for (int i = 0; i< 12; i++) u[i] = 0;

        // 重置模型状态
        currentModel = ModelType.Tinker2;
        isFallen = false;
        standUpStableCounter = 0;
        if (behaviorParameters != null && tinker2Model != null && !train)
        {
            behaviorParameters.Model = tinker2Model;
        }

        // 获取课程参数（训练模式下）
        if (train)
        {
            // 从Academy获取课程阶段参数
            // 使用Academy.Instance而不是FindObjectOfType
            if (Academy.Instance != null)
            {
                currentLesson = (int)Academy.Instance.EnvironmentParameters.GetWithDefault("lesson", 0f);
                // 根据课程阶段设置最大速度缩放
                // Lesson 0: 静态平衡 (maxSpeedScale = 0)
                // Lesson 1: 动态平衡 (0.3-0.6)
                // Lesson 2: 动态平衡 (0.5-0.8)
                // Lesson 3: 动态平衡 (0.7-1.0)
                // Lesson 4: 动态平衡 (0.9-1.3)
                // Lesson 5: 高速 + 踢球 (1.2-1.6)
                // Lesson 6: 超高速 + 踢球 (1.5-2.0)
                // Lesson 7: 超高速 + 踢球 + 弱干扰 (1.5-2.0)
                if (currentLesson == 0)
                {
                    maxSpeedScale = 0f;
                }
                else if (currentLesson == 1)
                {
                    maxSpeedScale = Random.Range(0.3f, 0.6f);
                }
                else if (currentLesson == 2)
                {
                    maxSpeedScale = Random.Range(0.5f, 0.8f);
                }
                else if (currentLesson == 3)
                {
                    maxSpeedScale = Random.Range(0.7f, 1.0f);
                }
                else if (currentLesson == 4)
                {
                    maxSpeedScale = Random.Range(0.9f, 1.3f);
                }
                else if (currentLesson == 5)
                {
                    maxSpeedScale = Random.Range(1.2f, 1.6f);
                }
                else
                {
                    // Lesson 6 & 7
                    maxSpeedScale = Random.Range(1.5f, 2.0f);
                }
            }
            else
            {
                // 如果没有Academy，使用默认值
                currentLesson = 0;
                maxSpeedScale = 0f;
            }
        }

        Quaternion randRot = rot0;
        float px = 0;
        float pz = 0;

        if(train)
        {
            randRot = rot0 * Quaternion.Euler(0, Random.Range(-180f,180f), 0);
            vr=0;
            wr=0;

            // 课程训练模式下，根据课程阶段设置初始速度
            // Lesson 0: 静态平衡，vr和wr保持为0
            if (currentLesson == 0)
            {
                vr = 0;
                wr = 0;
            }
            else
            {
                // Lesson 1+: 动态平衡，根据maxSpeedScale设置速度
                if(Random.Range(0,3)==0)vr = Random.Range(0.3f,0.6f) * maxSpeedScale * (Random.Range(0,2)*2-1);
                else if(Random.Range(0,2)==0) wr = Random.Range(0.3f,0.6f) * maxSpeedScale * (Random.Range(0,2)*2-1);
            }

            // cr = 0.5f;//Random.Range(0.1f,0.7f);
            cr = Random.Range(0.1f,0.7f);

            if(Random.Range(0,2)==0)
            {
                px = 4*(Random.Range(0,2)*2-1);
                pz = Random.Range(-4f,4f);
            }
            else
            {
                pz = 4*(Random.Range(0,2)*2-1);
                px = Random.Range(-4f,4f);
            }

        }
        else
        {
            vr=0;
            wr=0;
            cr=0.5f;
        }

        Vector3 randPos = new Vector3(pos0[0]+px, pos0[1], pos0[2]+pz);
        ObservationNum = 9 + 2 * ActionNum;
        if (fixbody) arts[0].immovable = true;
        if (keyboard)
        {
            randPos=pos0;
            randRot=rot0;
        }
        if (!fixbody)
        {
            arts[0].TeleportRoot(randPos, randRot);
            arts[0].velocity = Vector3.zero;
            arts[0].angularVelocity = Vector3.zero;
            arts[0].SetJointPositions(P0);
            arts[0].SetJointVelocities(W0);
        }

    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(EulerTrans(body.eulerAngles[0]) * 3.14f / 180f);
        sensor.AddObservation(EulerTrans(body.eulerAngles[2]) * 3.14f / 180f);
        sensor.AddObservation(body.InverseTransformDirection(arts[0].angularVelocity));
        sensor.AddObservation(body.InverseTransformDirection(arts[0].velocity));

        for (int i = 0; i < ActionNum; i++)
        {
            sensor.AddObservation(acts[i].jointPosition[0]);
            sensor.AddObservation(acts[i].jointVelocity[0]);
        }
        sensor.AddObservation(vr);
        sensor.AddObservation(wr);
        sensor.AddObservation(cr);
        sensor.AddObservation(Mathf.Sin(3.14f * 1 * tp / T1));
        sensor.AddObservation(Mathf.Cos(3.14f * 1 * tp / T1));
    }

    float EulerTrans(float eulerAngle)
    {
        if (eulerAngle <= 180)
            return eulerAngle;
        else
            return eulerAngle - 360f;
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        for (int i = 0; i < 12; i++) utotal[i] = 0;
        var continuousActions = actionBuffers.ContinuousActions;
        var kk = 0.9f;

        // 根据模型类型使用不同的系数
        float actionScale = (currentModel == ModelType.Tinker3) ? 2.5f : 2.0f;

        for (int i = 0; i < ActionNum; i++)
        {
            u[i] = u[i] * kk + (1 - kk) * continuousActions[i];
            utotal[i] = actionScale * kb[i] * u[i];
            if (fixbody) utotal[i] = 0;
        }


        int[] idx = new int[6] { -3, 4, 5, 8, -9, -10 };
        kb = new float[10] { 15, 30, 40, 15, 40,   15, 30, 40, 15, 40};
        T1 = 30;
        float d0 = cr*180f/3.14f;//10;
        float dh = 40;

        // Tinker3模型在起身时，dh应该为0
        if (currentModel == ModelType.Tinker3)
        {
            dh = 0;
            // 确保在起身模式下，vr和wr保持为0，让模型专注于起身
            vr = 0;
            wr = 0;
            cr = 0.5f;
        }
        else if(vr==0 && wr==0 && !fixbody)
        {

            if(tt>450 && tt<600)dh=0;
            if(tt>840)dh=0;
            if(keyboard)dh=0;
        }

        utotal[Mathf.Abs(idx[0]) - 1] += (dh * uf1 + d0) * Mathf.Sign(idx[0]);
        utotal[Mathf.Abs(idx[1]) - 1] += 2 * (dh * uf1 + d0) * Mathf.Sign(idx[1]);
        utotal[Mathf.Abs(idx[2]) - 1] += (dh * uf1 + d0) * Mathf.Sign(idx[2]);
        utotal[Mathf.Abs(idx[3]) - 1] += (dh * uf2 + d0) * Mathf.Sign(idx[3]);
        utotal[Mathf.Abs(idx[4]) - 1] += 2 * (dh * uf2 + d0) * Mathf.Sign(idx[4]);
        utotal[Mathf.Abs(idx[5]) - 1] += (dh * uf2 + d0) * Mathf.Sign(idx[5]);

        //utotal[1] = Mathf.Clamp(utotal[1], -200f, 0f);
        //utotal[7] = Mathf.Clamp(utotal[7], 0f, 200f);
        for (int i = 0; i < ActionNum; i++) SetJointTargetDeg(acts[i], utotal[i]);
    }
    void SetJointTargetDeg(ArticulationBody joint, float x)
    {
        var drive = joint.xDrive;
        drive.stiffness = 50f;
        drive.damping = 2f;
        //drive.forceLimit = 300f;
        drive.target = x;
        joint.xDrive = drive;
    }

    /// <summary>
    /// 执行踢球动作 - 可在训练和非训练模式下使用
    /// </summary>
    void PerformKickAction()
    {
        if (!IsValidJointIndex(L0_idx) || !IsValidJointIndex(L1_idx) ||
            !IsValidJointIndex(L2_idx) || !IsValidJointIndex(L3_idx) ||
            !IsValidJointIndex(R0_idx) || !IsValidJointIndex(R1_idx) ||
            !IsValidJointIndex(R2_idx) || !IsValidJointIndex(R3_idx))
        {
            return;
        }

        // 更新踢球计时器
        if (l_kick && !wait) tq += 2;
        if (r_kick && !wait) tr++;
        T2 = 40;

        // 设置默认腿部姿态
        SetJointTargetDeg(arts[L0_idx], 0);
        SetJointTargetDeg(arts[L1_idx], 0);
        SetJointTargetDeg(arts[L2_idx], 0);
        SetJointTargetDeg(arts[L3_idx], 0);

        SetJointTargetDeg(arts[R0_idx], 0);
        SetJointTargetDeg(arts[R1_idx], 0);
        SetJointTargetDeg(arts[R2_idx], 0);
        SetJointTargetDeg(arts[R3_idx], 0);

        // 左脚踢球动作
        if (tq > 0 && tq <= T2)
        {
            SetJointTargetDeg(arts[R0_idx], 0);
            SetJointTargetDeg(arts[R1_idx], 0);
            SetJointTargetDeg(arts[R2_idx], 0);
            SetJointTargetDeg(arts[R3_idx], 0);


            SetJointTargetDeg(arts[L0_idx], 0 + 0 * (-Mathf.Cos(3.14f * 2 * tq / T2) + 1) / 2f); // 髋关节外展
            SetJointTargetDeg(arts[L1_idx], 0 - 0 * (-Mathf.Cos(3.14f * 2 * tq / T2) + 1) / 2f); // 髋关节内旋
            SetJointTargetDeg(arts[L2_idx], 0 -70 * (-Mathf.Cos(3.14f * 2 * tq / T2) + 1) / 2f); // 后踢伸膝
            SetJointTargetDeg(arts[L3_idx], 0 - 90 * (-Mathf.Cos(3.14f * 2 * tq / T2) + 1) / 2f); // 前踢


        }
        if (tq >= T2)
        {
            tq = 0;
            l_kick = false;
        }

        // 右脚踢球动作
        if (tr > 0 && tr <= T2)
        {
            SetJointTargetDeg(arts[L0_idx], 0);
            SetJointTargetDeg(arts[L1_idx], 0);
            SetJointTargetDeg(arts[L2_idx], 0);
            SetJointTargetDeg(arts[L3_idx], 0);

            SetJointTargetDeg(arts[R0_idx], 0 + 0 * (-Mathf.Cos(3.14f * 2 * tr / T2) + 1) / 2f);
            SetJointTargetDeg(arts[R1_idx], 0 - 0 * (-Mathf.Cos(3.14f * 2 * tq / T2) + 1) / 2f);
            SetJointTargetDeg(arts[R2_idx], 0 + 70 * (-Mathf.Cos(3.14f * 2 * tq / T2) + 1) / 2f);
            SetJointTargetDeg(arts[R3_idx], 0 + 90 * (-Mathf.Cos(3.14f * 2 * tq / T2) + 1) / 2f);
        }
        if (tr >= T2)
        {
            tr = 0;
            r_kick = false;
            wait = true;
        }
    }

    /// <summary>
    /// 更新训练模式下的随机移动 - 根据课程阶段调整训练难度
    /// </summary>
    void UpdateTrainingMovement()
    {
        // Lesson 0: 静态平衡，不执行移动逻辑
        if (currentLesson == 0)
        {
            vr = 0;
            wr = 0;
            cr = 0.5f;
            return;
        }

        trainingMoveDuration++;

        // 每隔一定时间切换移动模式并生成新的速度值，或首次调用时初始化
        if (trainingMoveDuration >= trainingMoveChangeInterval || trainingMoveDuration == 1)
        {
            if (trainingMoveDuration >= trainingMoveChangeInterval)
            {
                // 根据课程阶段选择训练模式数量
                // Lesson 1-2: 基础动态平衡，使用较少的模式
                // Lesson 3+: 更复杂的移动模式
                int maxModes = (currentLesson <= 2) ? 5 : 10;
                trainingMoveMode = Random.Range(0, maxModes);
                trainingMoveDuration = 0;
            }

            // 根据模式设置移动参数（只在切换模式时生成一次）
            // 所有速度值都会根据maxSpeedScale进行缩放
            if (trainingMoveMode == 0)
            {
                // 纯前进 - 基础训练
                trainingVr = Random.Range(0.4f, 0.8f) * maxSpeedScale;
                trainingWr = 0;
            }
            else if (trainingMoveMode == 1)
            {
                // 纯转弯（原地转向）- 加强转向训练
                trainingVr = 0;
                trainingWr = Random.Range(0.5f, 1.0f) * maxSpeedScale * (Random.Range(0, 2) * 2 - 1);
            }
            else if (trainingMoveMode == 2)
            {
                // 快速转向（大角度差）- 针对性转向训练
                trainingVr = 0;
                trainingWr = Random.Range(0.7f, 1.2f) * maxSpeedScale * (Random.Range(0, 2) * 2 - 1);
            }
            else if (trainingMoveMode == 3)
            {
                // 前进+转弯（最常见）- 组合训练
                // 提高速度上限，训练高速转弯稳定性
                trainingVr = Random.Range(0.5f, 1.0f) * maxSpeedScale;
                trainingWr = Random.Range(0.3f, 0.7f) * maxSpeedScale * (Random.Range(0, 2) * 2 - 1);
            }
            else if (trainingMoveMode == 4)
            {
                // 前进+快速转弯 - 加强转向能力
                // 提高速度上限
                trainingVr = Random.Range(0.4f, 0.8f) * maxSpeedScale;
                trainingWr = Random.Range(0.6f, 1.0f) * maxSpeedScale * (Random.Range(0, 2) * 2 - 1);
            }
            else if (trainingMoveMode == 5)
            {
                // 后退+转弯 - 反向移动训练
                trainingVr = Random.Range(-0.5f, -0.3f) * maxSpeedScale;
                trainingWr = Random.Range(0.3f, 0.7f) * maxSpeedScale * (Random.Range(0, 2) * 2 - 1);
            }
            else if (trainingMoveMode == 6)
            {
                // 侧向移动训练（vr=0, wr较大）- 专门训练侧向移动能力
                trainingVr = 0;
                trainingWr = Random.Range(0.6f, 1.0f) * maxSpeedScale * (Random.Range(0, 2) * 2 - 1);
            }
            else if (trainingMoveMode == 7)
            {
                // 慢速前进+快速转向 - 模拟靠近球时的调整
                trainingVr = Random.Range(0.2f, 0.4f) * maxSpeedScale;
                trainingWr = Random.Range(0.7f, 1.1f) * maxSpeedScale * (Random.Range(0, 2) * 2 - 1);
            }
            else if (trainingMoveMode == 8)
            {
                // 交替转向训练 - 快速切换转向方向
                trainingVr = Random.Range(0.2f, 0.5f) * maxSpeedScale;
                // 随机选择转向方向，但保持较大的角速度
                trainingWr = Random.Range(0.5f, 0.9f) * maxSpeedScale * (Random.Range(0, 2) * 2 - 1);
            }
            else
            {
                // 静止平衡 - 减少静止时间，偶尔允许短暂静止
                // 有30%概率保持静止，70%概率给一个很小的速度或转向
                if (Random.Range(0f, 1f) < 0.3f)
                {
                    trainingVr = 0;
                    trainingWr = 0;
                }
                else
                {
                    // 给一个很小的速度或转向，不完全静止
                    if (Random.Range(0f, 1f) < 0.5f)
                    {
                        trainingVr = Random.Range(-0.1f, 0.1f) * maxSpeedScale;
                        trainingWr = 0;
                    }
                    else
                    {
                        trainingVr = 0;
                        trainingWr = Random.Range(-0.3f, 0.3f) * maxSpeedScale;
                    }
                }
            }
        }
        // 应用训练模式下的速度值（使用平滑过渡，提高过渡速度以增加难度）
        // 对于转向训练，使用更快的过渡速度
        float transitionSpeed = (Mathf.Abs(trainingWr) > 0.5f) ? Random.Range(0.01f, 0.2f) : Random.Range(0.005f, 0.06f);
        // vr = Mathf.MoveTowards(vr, trainingVr, transitionSpeed);
        vr = trainingVr;
        wr = Mathf.MoveTowards(wr, trainingWr, transitionSpeed);
        cr = 0.5f;

        // 根据课程阶段决定是否施加推力
        // Lesson 0-6: 不施加推力
        // Lesson 7: 施加弱推力
        if (currentLesson >= 7)
        {
            trainingForceTriggerTimer++;
            if (trainingForceTriggerTimer >= trainingForceInterval)
            {
                trainingForceTriggerTimer = 0;
                // 70%概率触发推力
                if (Random.Range(0f, 1f) < 0.7f)
                {
                    // Lesson 7: 弱推力范围 1f-3f
                    float minForce = 1f;
                    float maxForce = 3f;
                    float force = Random.Range(minForce, maxForce);

                    Vector3 randomForce = new Vector3(
                        Random.Range(-force, force),
                        0,
                        Random.Range(-force, force)
                    );

                    // 施加推力，使用Impulse模式以获得瞬时效果
                    arts[0].AddForce(randomForce, ForceMode.Impulse);
                }
                // 随机化下一次推力触发间隔，增加训练的不确定性
                trainingForceInterval = Random.Range(80, 150);
            }
        }
    }

    /// <summary>
    /// 训练模式下触发踢球动作 - 根据课程阶段控制踢球触发
    /// </summary>
    void CheckTrainingKickTrigger()
    {
        // Lesson 0-4: 不触发踢球，专注于平衡训练
        // Lesson 5+: 开始触发踢球动作
        if (currentLesson < 5)
        {
            return;
        }

        trainingKickTriggerTimer++;

        // 每隔一定时间检查是否触发踢球
        if (trainingKickTriggerTimer >= trainingKickInterval)
        {
            trainingKickTriggerTimer = 0;

            // 根据课程阶段调整踢球触发概率
            // Lesson 5: 30%概率触发踢球
            // Lesson 6+: 50%概率触发踢球
            float kickProbability = (currentLesson == 5) ? 0.3f : 0.5f;

            if (Random.Range(0f, 1f) < kickProbability && !wait && !l_kick && !r_kick)
            {
                // 随机选择左脚或右脚
                if (Random.Range(0, 2) == 0)
                {
                    l_kick = true;
                }
                else
                {
                    r_kick = true;
                }
            }
        }
    }

    /// <summary>
    /// 检测是否摔倒，如果摔倒则切换到Tinker3模型
    /// </summary>
    void CheckFallAndSwitchModel()
    {
        if (train) return;  // 训练模式下不切换模型

        float rollAngle = Mathf.Abs(EulerTrans(body.eulerAngles[0]));
        float pitchAngle = Mathf.Abs(EulerTrans(body.eulerAngles[2]));

        // 检测摔倒：角度超过阈值
        if (!isFallen && (rollAngle > fallAngleThreshold || pitchAngle > fallAngleThreshold))
        {
            isFallen = true;
            currentModel = ModelType.Tinker3;
            standUpStableCounter = 0;

            // 切换到Tinker3模型
            if (behaviorParameters != null && tinker3Model != null)
            {
                if (behaviorParameters.Model == tinker2Model) {
                    // 切换起身模型前：重置动作状态，避免残留动作影响起身
                    for (int i = 0; i < 12; i++)
                    {
                        u[i] = 0;
                        utotal[i] = 0;
                    }
                    // 在起身模式下，重置速度和目标
                    vr = 0;
                    wr = 0;
                    cr = 0.5f;
                    tq = 0;
                    tr = 0;
                    tw = 0;
                    // 重置踢球状态
                    l_kick = false;
                    r_kick = false;
                    wait = false;
                    behaviorParameters.Model = tinker3Model;
                    print($"[模型切换] 检测到摔倒，切换到Tinker3起身模型。角度: Roll={rollAngle:F1}°, Pitch={pitchAngle:F1}°");
                }
            }
            else
            {
                print($"[模型切换] 警告：无法切换到Tinker3模型！behaviorParameters={behaviorParameters != null}, tinker3Model={tinker3Model != null}");
            }

        }

        // 检测是否成功起身
        if (isFallen && currentModel == ModelType.Tinker3)
        {
            // 检查角度是否在可接受范围内
            if (rollAngle < standUpAngleThreshold && pitchAngle < standUpAngleThreshold)
            {
                standUpStableCounter++;

                // 保持稳定一定帧数后，认为成功起身
                if (standUpStableCounter >= standUpStableFrames)
                {
                    isFallen = false;
                    currentModel = ModelType.Tinker2;
                    standUpStableCounter = 0;

                    // 切换回Tinker2模型
                    if (behaviorParameters != null && tinker2Model != null)
                    {
                        behaviorParameters.Model = tinker2Model;
                        print($"[模型切换] 成功起身，切换回Tinker2追踪模型。角度: Roll={rollAngle:F1}°, Pitch={pitchAngle:F1}°");
                    }
                    else
                    {
                        print($"[模型切换] 警告：无法切换回Tinker2模型！behaviorParameters={behaviorParameters != null}, tinker2Model={tinker2Model != null}");
                    }
                }
            }
            else
            {
                // 如果角度又变大了，重置计数器
                standUpStableCounter = 0;
            }
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {

    }
    // 追踪球的逻辑（球复位逻辑已移到FixedUpdate中，确保无论使用哪个模型都能执行）
    void TrackBall() {
        if (currentModel == ModelType.Tinker3) return; // 起身模式下不追踪球
        if (ball != null)
        {
            // 球复位逻辑已移到FixedUpdate中，这里不再重复处理

            if(wasd && rival != null) ball = rival;
            Vector3 toBall = ball.position - body.position;
            toBall.y = 0;
            Vector3 toRival = rival != null ? rival.position - body.position : Vector3.zero;
            toRival.y = 0;

            Vector3 robotForward = body.forward;
            robotForward.y = 0;

            float angleDiff = Vector3.SignedAngle(robotForward.normalized, toBall.normalized, Vector3.up);


            // 使用平滑过渡，与训练模式保持一致，避免突然变化导致失衡
            // 靠近球时使用更快的响应速度
            float ballDistance = toBall.magnitude;
            float wrChangeSpeed = (ballDistance < 0.5f) ? 0.15f : 0.06f;  // 靠近球时加快响应
            float vrChangeSpeed = 0.1f; // 平滑过渡线速度
            float targetWr = Mathf.Clamp(angleDiff * 0.3f, -1f, 1f);
            float targetVr = 1f;


            // 踢球进球门逻辑：当距离球很近时，调整方向面向球门后射门
            // 优先确保球在机器人前方，然后再调整射门角度
            if (toBall.magnitude < 0.5f)
            {
                // 计算球门位置（如果goal为空，则使用默认位置 x=15.8）
                Vector3 goalPosition = goal != null ? goal.position : new Vector3(15.8f, 0f, 0f);
                Vector3 toGoal = goalPosition - body.position;
                toGoal.y = 0;

                // 计算到球门的角度差
                // SignedAngle(from, to, axis) 返回从from到to的角度
                // 如果to在from的右侧，返回正值（逆时针）；如果to在from的左侧，返回负值（顺时针）
                float angleToGoal = Vector3.SignedAngle(robotForward.normalized, toGoal.normalized, Vector3.up);

                // 计算球相对于机器人的角度差（用于确保球在正前方）
                float angleToBall = Vector3.SignedAngle(robotForward.normalized, toBall.normalized, Vector3.up);

                // 优先确保球在机器人正前方（角度差小于20度）
                if (Mathf.Abs(angleToBall) > 20f)
                {
                    // 球不在正前方，优先追踪球，保持球在正前方
                    // angleToBall: 正值表示球在右侧（需要逆时针转），负值表示球在左侧（需要顺时针转）
                    // 根据角度差动态调整转向速度：角度差越大，转向越快
                    float angleAbs = Mathf.Abs(angleToBall);
                    float dynamicTurnSpeed = Mathf.Clamp01(angleAbs / 45f);  // 角度差越大，系数越大（最大1.0）
                    float enhancedTargetWr = Mathf.Clamp(angleToBall * (0.5f + dynamicTurnSpeed * 0.5f), -1f, 1f);

                    // 验证转向方向：angleToBall的符号应该与enhancedTargetWr的符号一致
                    if (Mathf.Sign(enhancedTargetWr) != Mathf.Sign(angleToBall) && angleAbs > 1f)
                    {
                        print($"[警告] 追踪球时转向方向可能错误: 角度差={angleToBall:F1}°, 目标角速度={enhancedTargetWr:F2}");
                    }

                    wr = enhancedTargetWr;
                    vr = Mathf.MoveTowards(vr, -0.1f, vrChangeSpeed);  // 追踪球时保持中等速度
                }
                // 球已经在正前方，可以调整射门角度
                else if (Mathf.Abs(angleToGoal) > 20f)
                {
                    // 球在正前方，调整朝向球门
                    // Vector3.SignedAngle(from, to, axis) 返回从from到to的角度
                    // 返回值范围：-180° 到 180°
                    // 正值：to在from的右侧（需要逆时针旋转，wr>0）
                    // 负值：to在from的左侧（需要顺时针旋转，wr<0）
                    // 这已经是最短路径了，不需要额外归一化

                    // 直接使用angleToGoal，因为SignedAngle已经返回最短路径的角度
                    float normalizedAngle = angleToGoal;

                    // 验证角度范围（SignedAngle应该已经返回-180到180，但为了安全起见）
                    if (normalizedAngle > 180f) normalizedAngle -= 360f;
                    if (normalizedAngle < -180f) normalizedAngle += 360f;

                    // 根据角度差调整转向速度：角度差大时转向更快
                    float angleAbs = Mathf.Abs(normalizedAngle);
                    float turnSpeed = Mathf.Clamp01(angleAbs / 30f);

                    // 计算目标角速度：增加基础系数，并让turnSpeed的影响更大
                    float baseCoefficient = 0.6f;
                    float turnSpeedMultiplier = 1f + turnSpeed * 1.5f;
                    float targetWrToGoal = Mathf.Clamp(normalizedAngle * baseCoefficient * turnSpeedMultiplier, -1f, 1f);

                    // 验证转向方向是否正确
                    // 如果normalizedAngle > 0，球门在右侧，应该逆时针转（wr > 0）
                    // 如果normalizedAngle < 0，球门在左侧，应该顺时针转（wr < 0）
                    // targetWrToGoal的符号应该与normalizedAngle一致
                    if (Mathf.Sign(targetWrToGoal) != Mathf.Sign(normalizedAngle) && Mathf.Abs(normalizedAngle) > 1f)
                    {
                        // 如果符号不一致且角度较大，说明可能有问题，打印警告
                        print($"[警告] 转向方向可能错误: 角度差={normalizedAngle:F1}°, 目标角速度={targetWrToGoal:F2}, 球门位置={goalPosition}, 机器人位置={body.position}");
                    }

                    // 使用平滑过渡，但速度更快
                    wr = targetWrToGoal;
                    vr = Mathf.MoveTowards(vr, 0.6f, vrChangeSpeed);  // 调整方向时减速

                    // 调试信息：打印角度和方向（取消注释以查看详细信息）
                    // print($"[射门调整] 角度差: {normalizedAngle:F1}°, 目标角速度: {targetWrToGoal:F2}, 转向: {(targetWrToGoal > 0 ? "逆时针(右转)" : "顺时针(左转)")}, 球门位置: {goalPosition}, 机器人位置: {body.position}");
                }
                else
                {
                    // 球在正前方且已经面向球门，继续向前并准备射门
                    wr = 0;
                    vr = Mathf.MoveTowards(vr, 1.5f, vrChangeSpeed);

                    // 当角度差很小且距离球很近时，触发射门
                    if (Mathf.Abs(angleToGoal) < 15f && toBall.magnitude < 0.15f && !r_kick && !l_kick && !wait)
                    {
                        if (Random.Range(0, 3) == 1) r_kick = true;
                        else l_kick = true;
                        print($"触发射门：{l_kick}, {r_kick}");
                    }
                }
            }
            else
            {
                // 距离球较远时，使用原有的追踪逻辑
                wr = targetWr;
                vr = Mathf.MoveTowards(vr, targetVr, vrChangeSpeed);
            }

            if (rival != null)
            {
                angleDiff = Vector3.SignedAngle(robotForward.normalized, toRival.normalized, Vector3.up);

                if(Mathf.Abs(angleDiff)<15 && toRival.magnitude < 0.3f && r_kick==false && l_kick==false)
                {
                    // 踢球时也平滑过渡到0，而不是突然变为0
                    wr = Mathf.MoveTowards(wr, 0f, wrChangeSpeed);
                    if (Random.Range(0, 3) == 1) r_kick = true;
                    else l_kick = true;
                    print($"触发踢球：{l_kick}, {r_kick}");

                }
            }
        }
        if (Mathf.Abs(EulerTrans(body.eulerAngles[0])) > 70f || Mathf.Abs(EulerTrans(body.eulerAngles[2])) > 70f) EndEpisode();

        // 非训练模式的踢球逻辑：使用原有的踢球触发逻辑（通过rival检测等）
        PerformKickAction();
    }

    void FixedUpdate()
    {

        if(wait) tw++;
        if(tw>50)
        {
            wait=false;
            tw=0;
        }


        if (accelerate) Time.timeScale = 20;
        if (!accelerate) Time.timeScale = 1;

        tp++;
        if (tp > 0 && tp <= T1)
        {
            tp0 = tp;
            uf1 = (-Mathf.Cos(3.14f * 2 * tp0 / T1) + 1f) / 2f;
            uf2 = 0;
        }
        if (tp > T1 && tp <= 2 * T1)
        {
            tp0 = tp - T1;
            uf1 = 0;
            uf2 = (-Mathf.Cos(3.14f * 2 * tp0 / T1) + 1f) / 2f;
        }
        if (tp >= 2 * T1) tp = 0;

        tt++;
        if (tt > 900 && ko < 0.5f)
        {
            ko = 1f;
            print(222222222222222);
        }

        // 球复位逻辑（无论使用哪个模型都要执行）
        if (!train && ball != null)
        {
            if (Input.GetKey(KeyCode.Space) || Mathf.Abs(ball.position.z) > 6)
            {
                ball.position = posball0;
            }
            if (Mathf.Abs(ball.position.x) > 15.8)
            {
                ball.position = posball0;
                print(666);
                if (currentModel == ModelType.Tinker2)
                {
                    EndEpisode();
                }
            }
            // 按下 R 键复位机器人
            if (Input.GetKeyDown(KeyCode.R))
            {
                body.position = pos0;
                body.rotation = rot0;

                // 同时复位球
                // if (ball != null) ball.position = posball0;
                EndEpisode();
            }
        }

        // 检测摔倒并切换模型（仅在非训练模式下）
        if (!train)
        {
            CheckFallAndSwitchModel();

            // 只有在使用Tinker2模型时才执行追踪球的逻辑
            if (currentModel == ModelType.Tinker2)
            {
                TrackBall();
            } else {
                return;
            }
        } else {
            // 训练模式下，使用分离的方法更新随机移动
            UpdateTrainingMovement();

            // 训练模式下随机触发踢球
            CheckTrainingKickTrigger();


            if (l_kick || r_kick)
            {
                PerformKickAction();
            }
        }

        Vector3 randomForce=new Vector3(Random.Range(-1f, 1f),0,Random.Range(-1f, 1f));
        if(Random.Range(0, 100)==1 && disturb)arts[0].AddForce(2*randomForce, ForceMode.Impulse);

        var vel = body.InverseTransformDirection(arts[0].velocity);
        var wel = body.InverseTransformDirection(arts[0].angularVelocity);
        var live_reward = 1f;
        var ori_reward1 = -0.1f * Mathf.Abs(EulerTrans(body.eulerAngles[0]));
        var ori_reward2 = -0.1f * Mathf.Min(Mathf.Abs(body.eulerAngles[2]), Mathf.Abs(body.eulerAngles[2] - 360f));
        var wel_reward = 1 - 4*Mathf.Abs(wel[1] - wr);
        var vel_reward = 1 - 4*Mathf.Abs(vel[2] - vr) + 0*Mathf.Clamp(vel[2],-5f,1.5f) - Mathf.Abs(vel[0]);
        var reward = live_reward + (ori_reward1 + ori_reward2) * 1 +  wel_reward * 1 + vel_reward;

        AddReward(reward);
        float fallang=30f;
        if(train)fallang=20f;

        // 只有在训练模式下，或者使用Tinker2模型且严重摔倒时才结束Episode
        // 如果使用Tinker3模型，即使摔倒也不结束Episode，让它尝试起身
        if (train || currentModel == ModelType.Tinker2)
        {
            if (Mathf.Abs(EulerTrans(body.eulerAngles[0])) > fallang || Mathf.Abs(EulerTrans(body.eulerAngles[2])) > fallang)
            {
                //if(train)
                EndEpisode();
            }
        }
        if(train && tt>2000)EndEpisode();
    }

}

