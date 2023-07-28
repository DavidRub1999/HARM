function [MaxFlexNeck, MaxExtNeck, data1, data2, data3,MaxTurnedNeckRight,MaxTurnedNeckLeft,data4,data5,max_extension,data6,MaxRShoulderBackward,MaxRShoulderForward,MaxRShoulderSidewardRight,MaxRShoulderSidewardLeft,data7,MaxLShoulderBackward,MaxLShoulderForward,MaxLShoulderSidewardRight,MaxLShoulderSidewardLeft,data8,MaxBentLowerArmR,data9,MaxBentLowerArmL,data10,MaxPronLowerArmR,MaxSupLowerArmR,data11,MaxPronLowerArmL,MaxSupLowerArmL,data12,MaxFlexSideHandR,MaxExtSideHandR,data13,MaxFlexSideHandL,MaxExtSideHandL,data14,MaxFlexHandR,MaxExtHandR,data15,MaxFlexHandL,MaxExtHandL,data16] = postures(tree_data)

           tree = tree_data;
           
           

            %STEP 4A - Posture score head/neck and shoulder/upper arm
            % 4A-1. Head tilted forward >20° OR tilted backward >10°

            MaxFlexNeck = max(tree.jointData(6).jointAngle(:,3));
            
            MaxExtNeck =  -min(tree.jointData(6).jointAngle(:,3));
            

            %Percent Time that Head is tilted forward >20° OR tilted backward >10°
            countNeck10 = 0;
            countNeck20 = 0;

            for i = 1:size(tree.jointData(6).jointAngle,1)
                if (tree.jointData(6).jointAngle(i,3) > 20)
                    countNeck20 = countNeck20 + 1;
                elseif (tree.jointData(6).jointAngle(i,3) < (-10))
                    countNeck10 = countNeck10 + 1;
                end
            end

            percentageTimeNeck10_20 = ((countNeck10+countNeck20)/size(tree.jointData(6).jointAngle,1));
            data1 = percentageTimeNeck10_20*100;
            

            %4A-2. Head tilted sideward >20° OR turned >45°

            %Percent Time that Head is tilted sideward >20° OR turned >45°
            countNeck20_45 = 0;


            for i = 1:size(tree.jointData(6).jointAngle,1)
                if (tree.jointData(6).jointAngle(i,1) > 20)
                    countNeck20_45 = countNeck20_45 + 1;
                elseif (tree.jointData(6).jointAngle(i,1) < (-20))
                    countNeck20_45 = countNeck20_45 + 1;
                elseif (tree.jointData(6).jointAngle(i,2) >45)
                    countNeck20_45 = countNeck20_45 + 1;
                elseif (tree.jointData(6).jointAngle(i,2) <(-45))
                    countNeck20_45 = countNeck20_45 + 1;
                end
            end

            percentageTimeNeck20_45 = (countNeck20_45/size(tree.jointData(6).jointAngle,1));
            data2 = percentageTimeNeck20_45*100;
            


            %4A-3. Head tilted forward >20° AND turned >45°

            %Percent Time that Head is tilted Forward >20° AND turned >45°
            countNeckFT20_45 = 0;


            for i = 1:size(tree.jointData(6).jointAngle,1)
                if (tree.jointData(6).jointAngle(i,3) > 20) &&  (tree.jointData(6).jointAngle(i,2) > 45)
                    countNeckFT20_45 = countNeckFT20_45 + 1;
                elseif (tree.jointData(6).jointAngle(i,3) > 20) &&  (tree.jointData(6).jointAngle(i,2) < (-45))
                    countNeckFT20_45 = countNeckFT20_45 + 1;
                end
            end

            percentageTimeNeckFT20_45 = (countNeckFT20_45/size(tree.jointData(6).jointAngle,1));
            data3 = percentageTimeNeckFT20_45*100;
            


            %4A-4. Head tilted backward >10° AND turned >45°

            MaxTurnedNeckRight = max(tree.jointData(6).jointAngle(:,2));
            
            MaxTurnedNeckLeft =  -min(tree.jointData(6).jointAngle(:,2));
            

            %Percent Time that Head is tilted backward >10° AND turned >45°
            countNeckBT10_45 = 0;


            for i = 1:size(tree.jointData(6).jointAngle,1)
                if (tree.jointData(6).jointAngle(i,3) < (-10)) &&  (tree.jointData(6).jointAngle(i,2) > 45)
                    countNeckBT10_45 = countNeckBT10_45 + 1;
                elseif (tree.jointData(6).jointAngle(i,3) < (-10)) &&  (tree.jointData(6).jointAngle(i,2) < (-45))
                    countNeckBT10_45 = countNeckBT10_45 + 1;
                end
            end

            percentageTimeNeckBT10_45 = (countNeckBT10_45/size(tree.jointData(6).jointAngle,1));
            data4 = percentageTimeNeckBT10_45*100;
            

            %4A-5. Head (chin) pushed (extended) forward

            % Three points that define the plane
            countdistancehead = 0;
            max_ext = 0;
            for i = 1:size(tree.segmentData(6).position,1)
                p1 = tree.segmentData(6).position(i,:);
                p2 = tree.segmentData(8).position(i,:);
                p3 = tree.segmentData(12).position(i,:);

                % Fourth point to calculate distance to plane
                p4 = tree.segmentData(7).position(i,:);

                % Generate plane normal and constant
                v1 = p2 - p1;
                v2 = p3 - p1;
                normal = cross(v1, v2);
                d = -dot(normal, p1);

                % Calculate distance from p4 to plane
                distance = abs(dot(normal, p4) + d) / norm(normal);
                if distance>max_ext
                    max_ext= distance;
                end

                if (distance>0.03)
                    countdistancehead = countdistancehead + 1;
                end

            end

            %Percent time that Head (chin) pushed (extended) forward
            percentageTimeHeadForward = (countdistancehead)/size(tree.segmentData(6).position,1);
            data5 = percentageTimeHeadForward*100;
            
            max_extension = max_ext*100;
            

            %4A-6. (with arm unsupported) Upper arm forward >30° OR sideward >30° OR backwards >30° (Right Shoulder)

            %Percent (with  arm unsupported) Upper arm forward >30° OR sideward >30° OR backwards >30° (Right Shoulder)
            countRShoulder = 0;


            for i = 1:size(tree.jointData(8).jointAngle,1)
                if (tree.jointData(8).jointAngle(i,3) > 30)
                    countRShoulder = countRShoulder + 1;
                elseif (tree.jointData(8).jointAngle(i,3) < (-30))
                    countRShoulder = countRShoulder + 1;
                elseif (tree.jointData(8).jointAngle(i,1) > 30)
                    countRShoulder = countRShoulder + 1;
                end
            end

            percentageTimeRShoulder = (countRShoulder/size(tree.jointData(8).jointAngle,1));
            data6 = percentageTimeRShoulder*100;
            

            MaxRShoulderBackward = -min(tree.jointData(8).jointAngle(:,3));
            
            MaxRShoulderForward = max(tree.jointData(8).jointAngle(:,3));
            


            MaxRShoulderSidewardRight = max(tree.jointData(8).jointAngle(:,1));
            
            MaxRShoulderSidewardLeft =  -min(tree.jointData(8).jointAngle(:,1));
            

            %4A-6. (with arm unsupported) Upper arm forward >30° OR sideward >30° OR backwards >30° (Left Shoulder)

            % Percent (with  arm unsupported) Upper arm forward >30° OR sideward >30° OR backwards >30° (Left Shoulder)
            countLShoulder = 0;

            for i = 1:size(tree.jointData(12).jointAngle,1)
                if (tree.jointData(12).jointAngle(i,3) > 30)
                    countLShoulder = countLShoulder + 1;
                elseif (tree.jointData(12).jointAngle(i,3) < (-30))
                    countLShoulder = countLShoulder + 1;
                elseif (tree.jointData(12).jointAngle(i,1) > 30)
                    countLShoulder = countLShoulder + 1;
                end
            end

            percentageTimeLShoulder = (countLShoulder/size(tree.jointData(12).jointAngle,1));
            data7 = percentageTimeLShoulder*100;
            

            MaxLShoulderBackward = -min(tree.jointData(12).jointAngle(:,3));
            
            MaxLShoulderForward = max(tree.jointData(12).jointAngle(:,3));
            


            MaxLShoulderSidewardRight = max(tree.jointData(12).jointAngle(:,1));
            
            MaxLShoulderSidewardLeft =  -min(tree.jointData(12).jointAngle(:,1));
            

            %4A-7. Shoulders raised (high)
            countShoulderUP = 0;
            for i = 1:size(tree.segmentData(8).position,1)
                if (tree.segmentData(8).position(i,3) > (tree.segmentData(8).position(1,3)))
                    countShoulderUP = countShoulderUP + 1;
                end
            end
            percentageTimeShoulderUP = (countShoulderUP/size(tree.segmentData(8).position,1));
            data8 =  percentageTimeShoulderUP*100;
            


            %Step 4B - Posture score lower arm/wrist

            %4B-1. Elbow bent >T° OR extended >U°  (T=100º, U=60º) (Right Arm)

            MaxBentLowerArmR = max(tree.jointData(9).jointAngle(:,3));
            

            %Percent that Forearm is bent >T° OR extended >U°  (T=100º, U=60º) (Right Arm)
            countRForearm60_100 = 0;


            for i = 1:size(tree.jointData(9).jointAngle,1)
                if ((tree.jointData(9).jointAngle(i,3)) > 100)
                    countRForearm60_100 = countRForearm60_100 + 1;
                    %elseif ((tree.jointData(9).jointAngle(i,3)) <60)
                    %countRForearm60_100 = countRForearm60_100 + 1;
                end

            end

            percentageTimeRForearm60_100 = (countRForearm60_100/size(tree.jointData(9).jointAngle,1));
            data9 = percentageTimeRForearm60_100*100;
            

            %4B-1. Elbow bent >T° OR extended >U°  (T=100º, U=60º) (Left Arm)

            MaxBentLowerArmL = max(tree.jointData(13).jointAngle(:,3));
            


            %Percent that Forearm is bent >T° OR extended >U°  (T=100º, U=60º) (Left Arm)
            countLForearm60_100 = 0;


            for i = 1:size(tree.jointData(13).jointAngle,1)
                if ((tree.jointData(13).jointAngle(i,3)) > 100)
                    countLForearm60_100 = countLForearm60_100 + 1;
                    %elseif ((tree.jointData(13).jointAngle(i,3)) <60)
                    %countLForearm60_100 = countLForearm60_100 + 1;
                end

            end

            percentageTimeLForearm60_100 = (countLForearm60_100/size(tree.jointData(13).jointAngle,1));
            data10 = percentageTimeLForearm60_100*100;
            

            %4B-2. Lower arm rotate >40º from neutral (Right Arm)

            MaxPronLowerArmR = max(tree.jointData(9).jointAngle(:,2))-90;
            
            MaxSupLowerArmR=  -(min(tree.jointData(9).jointAngle(:,2))-90);
           

            %Percent that Forearm is rotated >40°
            countRForearm40 = 0;


            for i = 1:size(tree.jointData(9).jointAngle,1)
                if ((tree.jointData(9).jointAngle(i,2)-90) > 40)
                    countRForearm40 = countRForearm40 + 1;
                elseif ((tree.jointData(9).jointAngle(i,2)-90) <(-40))
                    countRForearm40 = countRForearm40 + 1;
                end

            end

            percentageTimeRForearm40 = (countRForearm40/size(tree.jointData(9).jointAngle,1));
            data11 = percentageTimeRForearm40*100;
            

            % 4B-2. Lower arm rotate >40º from neutral (Left Arm)

            MaxPronLowerArmL = max(tree.jointData(13).jointAngle(:,2))-90;
            
            MaxSupLowerArmL=  -(min(tree.jointData(13).jointAngle(:,2))-90);
            

            %Percent that Forearm is rotated >40°
            countLForearm40 = 0;


            for i = 1:size(tree.jointData(9).jointAngle,1)
                if ((tree.jointData(13).jointAngle(i,2)-90) > 40)
                    countLForearm40 = countLForearm40 + 1;
                elseif ((tree.jointData(13).jointAngle(i,2)-90) <(-40))
                    countLForearm40 = countLForearm40 + 1;
                end

            end

            percentageTimeLForearm40 = (countLForearm40/size(tree.jointData(13).jointAngle,1));
            data12 = percentageTimeLForearm40*100;
            

            %4B-3. The hand is bent sideways at the wrist >10° (Right Hand)
            MaxFlexSideHandR = max(tree.jointData(10).jointAngle(:,1));
            
            MaxExtSideHandR = max(tree.jointData(10).jointAngle(:,1));
            

            %Percent that Hand is bent >10°
            countRHand10 = 0;

            for i = 1:size(tree.jointData(10).jointAngle,1)
                if (tree.jointData(10).jointAngle(i,1) > 10)
                    countRHand10 = countRHand10 + 1;
                elseif (tree.jointData(10).jointAngle(i,1) <(-10))
                    countRHand10 = countRHand10 + 1;
                end

            end

            percentageTimeRHand10 = (countRHand10/size(tree.jointData(10).jointAngle,1));
            data13 = percentageTimeRHand10*100;
            

            %4B-3. The hand is bent sideways at the wrist >10° (Left Hand)
            MaxFlexSideHandL = max(tree.jointData(14).jointAngle(:,1));
            
            MaxExtSideHandL =  -min(tree.jointData(14).jointAngle(:,1));
            

            %Percent that Hand is bent >10°
            countLHand10 = 0;

            for i = 1:size(tree.jointData(14).jointAngle,1)
                if (tree.jointData(14).jointAngle(i,1) > 10)
                    countLHand10 = countLHand10 + 1;
                elseif (tree.jointData(14).jointAngle(i,1) <(-10))
                    countLHand10 = countLHand10 + 1;
                end

            end

            percentageTimeLHand10 = (countLHand10/size(tree.jointData(14).jointAngle,1));
            data14 = percentageTimeLHand10*100;
           

            %4B-4. The hand is bent at the wrist >15° (Right Hand)
            MaxFlexHandR = max(tree.jointData(10).jointAngle(:,3));
            
            MaxExtHandR =  -min(tree.jointData(10).jointAngle(:,3));
            

            %Percent that Hand is bent >15°
            countRHand15 = 0;


            for i = 1:size(tree.jointData(10).jointAngle,1)
                if (tree.jointData(10).jointAngle(i,3) > 15)
                    countRHand15 = countRHand15 + 1;
                elseif (tree.jointData(10).jointAngle(i,3) <(-15))
                    countRHand15 = countRHand15 + 1;
                end

            end

            percentageTimeRHand15 = (countRHand15/size(tree.jointData(10).jointAngle,1));
            data15 = percentageTimeRHand15*100;
            

            %4B-4. The hand is bent at the wrist >15° (Left Hand)

            MaxFlexHandL = max(tree.jointData(14).jointAngle(:,3));
            
            MaxExtHandL =  -min(tree.jointData(14).jointAngle(:,3));
            

            %Percent that Hand is bent >15°
            countLHand15 = 0;


            for i = 1:size(tree.jointData(14).jointAngle,1)
                if (tree.jointData(14).jointAngle(i,3) > 15)
                    countLHand15 = countLHand15 + 1;
                elseif (tree.jointData(14).jointAngle(i,3) <(-15))
                    countLHand15 = countLHand15 + 1;
                end

            end

            percentageTimeLHand15 = (countLHand15/size(tree.jointData(14).jointAngle,1));
            data16 = percentageTimeLHand15*100;
            
end