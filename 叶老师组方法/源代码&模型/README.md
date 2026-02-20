# AI驱动的足球机器人

## 一、机器人寻球、踢球

- 在进行机器人寻球之前，机器人的状态是站立的，通过按住键盘的wasd/↑↓←→键进行操作机器人。

- 通过修改其原始代码，把通过键盘按键控制的部分注释掉，采用以下的代码训练机器人寻球并踢球。

- ```c#
  using UnityEngine;
  using Unity.MLAgents;
  using Unity.MLAgents.Actuators;
  using Unity.MLAgents.Sensors;
  using Random = UnityEngine.Random;
  using System.Collections.Generic;
  using UnityEditor;
  using Unity.Sentis;
  using Unity.MLAgents.Policies;
  
  public class Tinker1Agent : Agent
  {
      int tp = 0;
      int tt = 0;
      int tp0 = 0;
      float kv = 0.4f;
      float uf1 = 0;
      float uf2 = 0;
      float[] u = new float[12] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
      float[] utotal = new float[12] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
      Transform body;
      
      public Transform ball;
      public float maxBallDistance = 10f;
      public bool fixbody = false;
      public bool train;
      public bool accelerate;
      public int ObservationNum;
      public int ActionNum;
  
      [Header("神经网络")]
      public ModelAsset standUp;
      public ModelAsset toBall;
  
      List<float> P0 = new List<float>();
      List<float> W0 = new List<float>();
      List<Transform> bodypart = new List<Transform>();
      Vector3 pos0;
      Vector3 posb0;
      Quaternion rot0;
      ArticulationBody[] arts = new ArticulationBody[40];
      ArticulationBody[] acts = new ArticulationBody[12];
  
      public float ko = 1;
      public float vr = 0;
      public float wr = 0;
      public bool wasd = false;
      float currentWr = 0;
      float currentVr = 0;
  
      public Transform goalLeft;
      public Transform goalRight;
  
      public override void Initialize()
      {
          var behaviorParams = this.GetComponent<BehaviorParameters>();
          //behaviorParams.Model = toBall;
  
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
          body = arts[0].GetComponent<Transform>();
          pos0 = body.position;
          rot0 = body.rotation;
          arts[0].GetJointPositions(P0);
          arts[0].GetJointVelocities(W0);
          accelerate = train;
      }
  
  
      private bool _isClone = false; 
      void Start()
      {
          // 获取球门引用
          goalLeft = GameObject.FindGameObjectWithTag("purpleGoal").transform;
          goalRight = GameObject.FindGameObjectWithTag("blueGoal").transform;
          Time.fixedDeltaTime = 0.01f;
          if (train && !_isClone) 
          {
              for (int i = 1; i < 24; i++)
              {
                  GameObject clone = Instantiate(gameObject); 
                  clone.name = $"{name}_Clone_{i}"; 
                  clone.GetComponent<Tinker1Agent>()._isClone = true; 
              }
          }
      }
      void ChangeLayerRecursively(GameObject obj, int targetLayer)
      {
          obj.layer = targetLayer;
          foreach (Transform child in obj.transform)ChangeLayerRecursively(child.gameObject, targetLayer);
      }
      private float previousDistanceToBall;
  
      public override void OnEpisodeBegin()
      {
          if (ball != null)
          {
              previousDistanceToBall = Vector3.Distance(body.position, ball.position); // 初始化上一帧的距离
          }
  
          tp = 0;
          tt = 0;
          for (int i = 0; i< 12; i++) u[i] = 0;
          ObservationNum = 9 + 2 * ActionNum;
          if (fixbody) arts[0].immovable = true;
          if (!fixbody)
          {
              arts[0].TeleportRoot(pos0, rot0);
              arts[0].velocity = Vector3.zero;
              arts[0].angularVelocity = Vector3.zero;
              arts[0].SetJointPositions(P0);
              arts[0].SetJointVelocities(W0);
          }
          vr = 0;
          wr = 0;
          if(Random.Range(0,2)==1)vr = Random.Range(0f,0.6f);
          else wr = Random.Range(-1f,1f);
      }
  
      public override void CollectObservations(VectorSensor sensor)
      {
          if (ball != null)
          {
              Vector3 ballRelativePosition = body.InverseTransformPoint(ball.position);
              Vector3 ballRelativeVelocity = body.InverseTransformDirection(ball.GetComponent<Rigidbody>().velocity);
              sensor.AddObservation(ballRelativePosition);
              sensor.AddObservation(ball.GetComponent<Rigidbody>().velocity);
              sensor.AddObservation(Vector3.Distance(body.position, ball.position));
          }
          else
          {
              sensor.AddObservation(Vector3.zero);
              sensor.AddObservation(Vector3.zero);
              sensor.AddObservation(0f);
          }
  
          // 新增：脚部与球的相对位置（帮助判断踢球时机）
          if (ball != null)
          {
              Transform leftFoot = acts[2].transform; 
              Transform rightFoot = acts[5].transform; 
              Vector3 leftFootToBall = ball.position - leftFoot.position;
              Vector3 rightFootToBall = ball.position - rightFoot.position;
              sensor.AddObservation(leftFootToBall.magnitude);
              sensor.AddObservation(rightFootToBall.magnitude);
              sensor.AddObservation(body.InverseTransformDirection(leftFootToBall.normalized));
              sensor.AddObservation(body.InverseTransformDirection(rightFootToBall.normalized));
          }
          else
          {
              sensor.AddObservation(0f);
              sensor.AddObservation(0f);
              sensor.AddObservation(Vector3.zero);
              sensor.AddObservation(Vector3.zero);
          }
  
          sensor.AddObservation(body.InverseTransformDirection(Vector3.down));
          sensor.AddObservation(body.InverseTransformDirection(arts[0].angularVelocity));
          sensor.AddObservation(body.InverseTransformDirection(arts[0].velocity));
          for (int i = 0; i < ActionNum; i++)
          {
              sensor.AddObservation(acts[i].jointPosition[0]);
              sensor.AddObservation(acts[i].jointVelocity[0]);
          }
          sensor.AddObservation(vr);
          sensor.AddObservation(wr);
  
          // 【新增：球门的观测】
          // 球门相对于机器人的位置、距离
          if (goalLeft != null && goalRight != null)
          {
              Vector3 goalLeftRelative = body.InverseTransformPoint(goalLeft.position);
              Vector3 goalRightRelative = body.InverseTransformPoint(goalRight.position);
              sensor.AddObservation(goalLeftRelative);   // 球门左相对于机器人的位置
              sensor.AddObservation(goalRightRelative);  // 球门右相对于机器人的位置
              sensor.AddObservation(Vector3.Distance(body.position, goalLeft.position)); // 到左球门距离
              sensor.AddObservation(Vector3.Distance(body.position, goalRight.position));// 到右球门距离
          }
          else
          {
              // 若球门未找到，填充默认值
              sensor.AddObservation(Vector3.zero);
              sensor.AddObservation(Vector3.zero);
              sensor.AddObservation(0f);
              sensor.AddObservation(0f);
          }
  
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
          float[] kb = new float[12] { 10, 10, 30, 50, 30,     10, 10, 30, 50, 30, 0, 0 };
          for (int i = 0; i < ActionNum; i++)
          {
              u[i] = u[i] * kk + (1 - kk) * continuousActions[i];
              utotal[i] = kb[i] * u[i];
              if (fixbody) utotal[i] = 0;
          }
  
          int[] idx = new int[6] { -2, -3, 4, 7, 8, -9 };
          float d0 = 30;
          float dh = 20;
          utotal[Mathf.Abs(idx[0])] += (dh * uf1 + d0) * Mathf.Sign(idx[0]);
          utotal[Mathf.Abs(idx[1])] -= 2 * (dh * uf1 + d0) * Mathf.Sign(idx[1]);
          utotal[Mathf.Abs(idx[2])] += (dh * uf1 + d0) * Mathf.Sign(idx[2]);
          utotal[Mathf.Abs(idx[3])] += (dh * uf2 + d0) * Mathf.Sign(idx[3]);
          utotal[Mathf.Abs(idx[4])] -= 2 * (dh * uf2 + d0) * Mathf.Sign(idx[4]);
          utotal[Mathf.Abs(idx[5])] += (dh * uf2 + d0) * Mathf.Sign(idx[5]);
  
          for (int i = 0; i < ActionNum; i++) SetJointTargetDeg(acts[i], utotal[i]);
      }
      void SetJointTargetDeg(ArticulationBody joint, float x)
      {
          var drive = joint.xDrive;
          drive.stiffness = 2000f;
          drive.damping = 100f;
          drive.forceLimit = 300f;
          drive.target = x;
          joint.xDrive = drive;
      }
      public override void Heuristic(in ActionBuffers actionsOut)
      {
          
      }
  
      void FixedUpdate()
      {
          if (!train)
          {
              vr = currentVr;
              wr = currentWr;
          }
          
          if (accelerate) Time.timeScale = 20;
          if (!accelerate) Time.timeScale = 1;
          tp++;
          int T1 = 25;
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
          ko = 0.1f;
          
          tt++;
          if (tt > 900 && kv < 0.5f)
          {
              kv = 0.7f;
              print(222222222222222);
          }
  
          // 【新增：进球检测与奖励】
          if (IsBallInGoalLeft())
          {
              AddReward(10f); // 进球奖励（数值越大，优先级越高）
              EndEpisode();   // 进球后结束本轮训练
          }
          else if (IsBallInGoalRight())
          {
              AddReward(10f);
              EndEpisode();
          }
  
          float totalReward = CalculateImprovedReward();
          AddReward(totalReward * Time.fixedDeltaTime);
          if (Mathf.Abs(EulerTrans(body.eulerAngles[0])) > 20f || Mathf.Abs(EulerTrans(body.eulerAngles[2])) > 20f)
          {
              if(train)EndEpisode();
          }
      }
      private float CalculateImprovedReward()
      {
          float reward = 0f;
          // 1. 前进奖励
          Vector3 forwardVelocity = body.InverseTransformDirection(arts[0].velocity);
          float forwardSpeed = forwardVelocity.z;
          float forwardReward = Mathf.Clamp(forwardSpeed, 0, 1f) * 0.1f;
          reward += forwardReward;
          // 2. 生存奖励
          reward += 0.001f;
          // 3. 稳定性奖励
          float pitchAngle = Mathf.Abs(EulerTrans(body.eulerAngles[0]));
          float rollAngle = Mathf.Abs(EulerTrans(body.eulerAngles[2]));
          float stabilityPenalty = -0.005f * (Mathf.Pow(pitchAngle / 180f, 2) + Mathf.Pow(rollAngle / 180f, 2));
          reward += stabilityPenalty;
          // 4. 能量效率惩罚
          float energyPenalty = 0f;
          for (int i = 0; i < ActionNum; i++)
          {
              energyPenalty += Mathf.Abs(u[i]) * 0.0005f;
          }
          reward -= energyPenalty;
          float soccerReward = CalculateSoccerReward();
          reward += soccerReward;
  
          return reward;
      }
      private float CalculateSoccerReward()
      {
          float soccerReward = 0f;
          if (ball == null) return soccerReward;
          // 1. 持续接近奖励
          float currentDistanceToBall = Vector3.Distance(body.position, ball.position);
          // 2.归一化距离奖励
          float maxEffectiveDistance = 3f;
          float effectiveDistance = Mathf.Min(currentDistanceToBall, maxEffectiveDistance);
          float distanceReward = (1.0f - effectiveDistance / maxEffectiveDistance) * 0.3f;
          soccerReward += distanceReward;
          float deltaDistance = previousDistanceToBall - currentDistanceToBall;
          float progressReward = deltaDistance * 2.0f;
          soccerReward += distanceReward;
          // 3. 接近接触点的额外奖励
          if (currentDistanceToBall < 1.5f)
          {
              float proximityBonus = (1.5f - currentDistanceToBall) * 0.8f;
              soccerReward += proximityBonus;
          }
          // 4. 方向奖励
          Vector3 toBall = (ball.position - body.position).normalized;
          float dotProduct = Vector3.Dot(body.forward, toBall);
          float orientationReward = dotProduct * 0.1f;
          soccerReward += orientationReward;
  
          if (currentDistanceToBall < 1.0f && currentDistanceToBall > 0.3f)
          {
              // 计算"犹豫惩罚"：在球旁边徘徊会被惩罚
              float hesitationPenalty = -0.1f * Time.fixedDeltaTime;
              soccerReward += hesitationPenalty;
          }
  
          // 接触持续奖励（当非常接近时）
          if (currentDistanceToBall < 0.5f)
          {
              float contactReward = 0.2f * Time.fixedDeltaTime;
              soccerReward += contactReward;
          }
  
          // 【新增：踢球动作奖励】
          float kickReward = 0f;
          for (int i = 0; i < ActionNum; i++)
          {
              float jointSpeed = acts[i].jointVelocity[0]; // 关节角速度（需匹配关节类型）
              if (jointSpeed > 2f) // 设定阈值：角速度>2f认为是踢球动作
              {
                  kickReward += jointSpeed * 0.01f; // 角速度越快，奖励越高
              }
          }
          // 【新增：朝向球门奖励】
          if (goalLeft != null && goalRight != null)
          {
              Vector3 goalCenter = (goalLeft.position + goalRight.position) / 2f; // 球门中心
              Vector3 toGoal = (goalCenter - body.position).normalized; // 机器人→球门的方向
              float orientationToGoal = Vector3.Dot(body.forward, toGoal); // 机器人前进方向与“向球门”的点积
              float goalOrientationReward = orientationToGoal * 0.1f; // 点积越大，奖励越高
              soccerReward += goalOrientationReward;
          }
          soccerReward += kickReward;
  
          previousDistanceToBall = currentDistanceToBall;
          return soccerReward;
      }
      private void OnCollisionEnter(Collision collision)
      {
          if (train && collision.gameObject.CompareTag("ball"))
          {
              float baseKickReward = 2.0f;
              Vector3 kickDirection = (collision.contacts[0].point - body.position).normalized;
              float forwardEffectiveness = Mathf.Clamp01(Vector3.Dot(body.forward, kickDirection));
              float kickSpeed = collision.relativeVelocity.magnitude;
              float qualityBonus = forwardEffectiveness * kickSpeed * 0.5f;
              float totalKickReward = baseKickReward + qualityBonus;
              AddReward(totalKickReward);
          }
      }
      // 新增踢球门
      private bool IsBallInGoalLeft()
      {
          if (ball == null) return false;
          float x = ball.position.x;
          float z = ball.position.z;
          return x > 15f && Mathf.Abs(z) < 17f;
      }
  
      private bool IsBallInGoalRight()
      {
          if (ball == null) return false;
          float x = ball.position.x;
          float z = ball.position.z;
          return x > -17f && Mathf.Abs(z) < -15f;
      }
  }
  
  ```

  - 在训练完成后，得到的是tiqiu.onnx神经网络模型，将其拖入到Model里面，可以进行测试。

  - 测试完成之后，在此基础上加入倒地起身的逻辑。

    ```c#
        void FixedUpdate()
        {
            if (!train)
            {
                vr = currentVr;
                wr = currentWr;
            }
    
            if (accelerate) Time.timeScale = 20;
            if (!accelerate) Time.timeScale = 1;
            tp++;
            int T1 = 25;
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
            ko = 0.1f;
    
            tt++;
            if (tt > 900 && kv < 0.5f)
            {
                kv = 0.7f;
                print(222222222222222);
            }
            // 【新增：进球检测与奖励】
            if (IsBallInGoalLeft())
            {
                AddReward(10f); // 进球奖励（数值越大，优先级越高）
                EndEpisode();   // 进球后结束本轮训练
            }
            else if (IsBallInGoalRight())
            {
                AddReward(10f);
                EndEpisode();
            }
    
            float totalReward = CalculateImprovedReward();
            AddReward(totalReward * Time.fixedDeltaTime);
            /*摔倒后结束回合*/
            if (Mathf.Abs(EulerTrans(body.eulerAngles[0])) > 20f || Mathf.Abs(EulerTrans(body.eulerAngles[2])) > 20f)
            {
                if (train) EndEpisode();
            }
            //新增：运行时进行神经网络切换，在切换点加日志，看有没有成功换模型
            //bool fallen = Mathf.Abs(EulerTrans(body.eulerAngles.x)) > 10f ||
            //              Mathf.Abs(EulerTrans(body.eulerAngles.z)) > 10f;
            bool singleFall = Vector3.Angle(body.up, Vector3.up) > 25f   // 躯干与竖直方向夹角 > 25°
               || body.position.y < -0.505f;                   // 髋部高度低于角色高度
            //int fallConsecutive = 0;
            //const int FALL_MIN_CONSEC = 6;   // 连续几帧都异常才认为摔倒
            bool fallen = false;
            if (singleFall)
            {
                fallConsecutive++;
                if (fallConsecutive >= FALL_MIN_CONSEC) fallen = true;
            }
            else
            {
                fallConsecutive = 0;   // 一旦站直就重置
            }
            if (fallen && !wasFallen)          // 刚倒下
            {
                var behaviorParams = GetComponent<BehaviorParameters>();
                behaviorParams.Model = standUp;  // 关键修复点
    
                SetModel(STAND_NAME, standUp);
                Debug.Log($"[{Time.frameCount}] 切换到 STAND 模型");
                System.Array.Clear(utotal, 0, utotal.Length);
                System.Array.Clear(u, 0, u.Length);
                isStandingUp = true;
                standUpTimer = 0;              // 重置冷却
            }
            else if (!fallen && isStandingUp)  // 已站直
            {
                standUpTimer++;
                Debug.Log($"StandUpTimer {standUpTimer}/{STAND_UP_FRAMES}");
                if (standUpTimer >= STAND_UP_FRAMES)   // 满 1.5 秒才允许切回
                {
                    SetModel(SEEK_NAME, toBall);
                    isStandingUp = false;
                    standUpTimer = 0;
                }
            }
            wasFallen = fallen;
        }
    ```

    - 写完切换逻辑之后，把相应的模型都拖入对应的Models框中。

    - 修改机器人采取动作的代码。

      ```c#
      public override void OnActionReceived(ActionBuffers actionBuffers)
      {
          for (int i = 0; i < 12; i++) utotal[i] = 0;
          var continuousActions = actionBuffers.ContinuousActions;
          var kk = 0.9f;
          // 机器人摔倒
          if (isStandingUp)
          {
              float[] kb = new float[10] { 15, 30, 40, 15, 40, 15, 30, 40, 15, 40 };
              for (int i = 0; i < ActionNum; i++)
              {
                  u[i] = u[i] * kk + (1 - kk) * continuousActions[i];
                  utotal[i] = 2.5f * kb[i] * u[i];
                  if (fixbody) utotal[i] = 0;
              }
              int[] idx = new int[6] { -3, 4, 5, 8, -9, -10 };
      
              T1 = 30;
              float d0 = 0.5f * 180f / 3.14f;
              float dh = 40;
              dh = 0;
              {
                  utotal[Mathf.Abs(idx[0]) - 1] += (dh * uf1 + d0) * Mathf.Sign(idx[0]);
                  utotal[Mathf.Abs(idx[1]) - 1] += 2 * (dh * uf1 + d0) * Mathf.Sign(idx[1]);
                  utotal[Mathf.Abs(idx[2]) - 1] += (dh * uf1 + d0) * Mathf.Sign(idx[2]);
                  utotal[Mathf.Abs(idx[3]) - 1] += (dh * uf2 + d0) * Mathf.Sign(idx[3]);
                  utotal[Mathf.Abs(idx[4]) - 1] += 2 * (dh * uf2 + d0) * Mathf.Sign(idx[4]);
                  utotal[Mathf.Abs(idx[5]) - 1] += (dh * uf2 + d0) * Mathf.Sign(idx[5]);
              }
              for (int i = 0; i < ActionNum; i++) SetJointTargetDegStand(acts[i], utotal[i]);
          }
          //机器人寻球
          else
          {
              float[] kb = new float[12] { 10, 10, 30, 50, 30, 10, 10, 30, 50, 30, 0, 0 };
              for (int i = 0; i < ActionNum; i++)
              {
                  u[i] = u[i] * kk + (1 - kk) * continuousActions[i];
                  utotal[i] = kb[i] * u[i];
                  if (fixbody) utotal[i] = 0;
              }
              int[] idx = new int[6] { -2, -3, 4, 7, 8, -9 };
              float d0 = 30;
              float dh = 20;
              utotal[Mathf.Abs(idx[0])] += (dh * uf1 + d0) * Mathf.Sign(idx[0]);
              utotal[Mathf.Abs(idx[1])] -= 2 * (dh * uf1 + d0) * Mathf.Sign(idx[1]);
              utotal[Mathf.Abs(idx[2])] += (dh * uf1 + d0) * Mathf.Sign(idx[2]);
              utotal[Mathf.Abs(idx[3])] += (dh * uf2 + d0) * Mathf.Sign(idx[3]);
              utotal[Mathf.Abs(idx[4])] -= 2 * (dh * uf2 + d0) * Mathf.Sign(idx[4]);
              utotal[Mathf.Abs(idx[5])] += (dh * uf2 + d0) * Mathf.Sign(idx[5]);
              for (int i = 0; i < ActionNum; i++) SetJointTargetDeg(acts[i], utotal[i]);
          }
      }
      
      void SetJointTargetDeg(ArticulationBody joint, float x)
      {
          var drive = joint.xDrive;
          drive.stiffness = 2000f;
          drive.damping = 100f;
          drive.forceLimit = 300f;
          drive.target = x;
          joint.xDrive = drive;
      }
      
      void SetJointTargetDegStand(ArticulationBody joint, float x)
      {
          var drive = joint.xDrive;
          drive.stiffness = 50f;
          drive.damping = 2f;
          drive.target = x;
          joint.xDrive = drive;
      }
      ```

      - 在修改完成这个之后基本上就可以了，运行之后机器人可以在踢球的过程中进行倒地起身，之后并继续踢球。
      - **以上内容可以详细看源代码。**

## 二、机器人倒地起身

在格物平台中的StandUp.unity中可以对机器人进行倒地起身训练，基于上述机器人踢球的训练，在这个倒地起身场景中需要增加机器人的观测值到58。

```c#
    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(EulerTrans(body.eulerAngles[0])*3.14f/180f);
        sensor.AddObservation(EulerTrans(body.eulerAngles[2])*3.14f/180f);
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

        //增加：补25个0占位，把33变成58
        for (int i = 0; i < 25; i++)
            sensor.AddObservation(0f);
    }
```

- 在修改好代码之后，来带unity界面，修改inspector中的SpaceSize为58，并可以开始进行训练，需要勾选train。

```bash
mlagents-learn config.yaml --run-id=standUp --force
```


- 在训练过程中，需要全选克隆出来的机器人，然后逐步降低Fy（力的大小）的数值，初始设置为70，在训练过程中，依据奖励数值大小来减少Fy，大约当平均奖励达到500左右，就可以将Fy减少5（不一定是5，根据自己的需求来）。
- 训练结束可以得到standUp.onnx神经网络模型。
- 在inspector界面的Model可以选择刚刚训练好的神经网络模型进行测试，取消勾选train，点击运行按钮。

- **以上内容可以详细看源代码。**
