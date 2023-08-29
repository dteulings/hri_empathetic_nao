#data analysis for HRI

library(car)
library(ggplot2)
library(tidyr)
library(pastecs)
library(ltm)
# input the data
data <- read.csv("C:\\Users\\Zizi\\Desktop\\master\\Human Robot Interaction\\project\\numeric.csv", header = TRUE, stringsAsFactors = FALSE)
head(data)
nr_part = nrow(data) - 2
questions <- data[1,18:ncol(data)] ;questions
data <- data[3:(3+nr_part-1),18:ncol(data)]

names <- names(data);names
new_data <- sapply(data[names[2:ncol(data)]],as.numeric)
data <- cbind(new_data, data[names[1]])
data
data[data$Q25 == 2, "Q25"] <- 'base'
data[data$Q25 == 1, "Q25"] <- 'mirror'
#one of the participants entered the wrong condition in the questionnaire.
#however we were able to backtrack which one it was.
data[7,"Q25"] <- "base"

data["Q25"] <- sapply(data["Q25"],as.factor)

which(is.na.data.frame(data))
#we decided to replace all the NAs with 1: strongly disagree, since that was
#the default value. We suspect that the participants did not move it and the
# value did not register because of that.
data[8,c("Q1_2","Q1_5")] <- 1
data[6,"Q9_1"] <- 1

#sum and normalize all scores

# merge scores from separate questions to categories and normalise them by
# taking the average
questions
empathy <- data[ , 4:9];empathy
emp_mirror <- data[data$Q25=="mirror", 4:9];emp_mirror
emp_base <- data[data$Q25=="base", 4:9];emp_base

anthropomorphism <- data[ , 10:14]
ant_mirror <-data[data$Q25=="mirror", 10:14];
ant_base <- data[data$Q25=="base", 10:14];

animacy <- data[ , 15:19];animacy
ani_mirror <- data[data$Q25=="mirror", 15:19];
ani_base <- data[data$Q25=="base", 15:19];

likeability <- data[ , 20:24]
lik_mirror <- data[data$Q25=="mirror", 20:24];
lik_base <- data[data$Q25=="base", 20:24];

intelligence <- data[ ,25: 29 ]
int_mirror <- data[data$Q25=="mirror", 25: 29];
int_base <- data[data$Q25=="base", 25: 29];

understanding <- data[ ,30: 32 ]
und_mirror <- data[data$Q25=="mirror", 30: 32];
und_base <- data[data$Q25=="base", 30: 32];

empathy_avg <- rowMeans(empathy, na.rm=TRUE);empathy_avg
anthropomorphism_avg <- rowMeans(anthropomorphism, na.rm=TRUE);anthropomorphism_avg
animacy_avg <- rowMeans(animacy, na.rm=TRUE);animacy_avg
likeability_avg <- rowMeans(likeability, na.rm=TRUE);likeability_avg
intelligence_avg <- rowMeans(intelligence, na.rm=TRUE)
understanding_avg <-rowMeans(understanding, na.rm=TRUE);understanding_avg

averaged_data <- data.frame(
    empathy = empathy_avg,
    anthropomorphism = anthropomorphism_avg,
    animacy = animacy_avg,
    likeability = likeability_avg,
    intelligence= intelligence_avg,
    understanding = understanding_avg,
    condition =  as.vector(data[,"Q25"])
)
averaged_data



# plot the data
emp_plot <- ggplot(averaged_data, aes(condition, empathy))+ geom_boxplot() + geom_point() + ylim(1,5)
emp_plot

anth_plot <- ggplot(averaged_data, aes(condition, anthropomorphism))+ geom_boxplot() + geom_point()+ geom_point() + ylim(1,5)
anth_plot

anim_plot <- ggplot(averaged_data, aes(condition, animacy))+ geom_boxplot() + geom_point()+ geom_point() + ylim(1,5)
anim_plot

lik_plot <- ggplot(averaged_data, aes(condition, likeability))+ geom_boxplot() + geom_point()+ geom_point() + ylim(1,5)
lik_plot

intel_plot <- ggplot(averaged_data, aes(condition, intelligence))+ geom_boxplot() + geom_point()+ geom_point() + ylim(1,5)
intel_plot

underst_plot <- ggplot(averaged_data, aes(condition, understanding))+ geom_boxplot() + geom_point()+ geom_point() + ylim(1,5)
underst_plot
# we suspect that the extent to which NAO was understood by the participant, can
# have an influence on the way it is perceived.
#for this reason, we also want to check whether there was a difference between 
# the groups in understanding.

# check the questionnaiore consistence for the categories,
# especially the empathy questions
# cronbach's alpha should be higher than .7 because it concerns a psychological 
# construct and therefore can be taken broadly.  
# report as a measure of reliability together with (\alpha = .x) 
# drop items if their raw alpha is greater than the current alpha, since
# that means that the consistency is better without those questions.
# then redo the analysis

#intelligence
psych::alpha(int_mirror) #74
psych::alpha(int_base)# 98

#anthropomorphism
psych::alpha(ant_mirror)#0.12 :(
psych::alpha(ant_base) #0.83
#ANIMACY
psych::alpha(ani_mirror) #0.58 :(
psych::alpha(ani_base) #0.76
#likeability
psych::alpha(lik_mirror) #86
psych::alpha(lik_base) #87
#understanding
psych::alpha(und_mirror) #79
psych::alpha(und_base) #89

#the godspeed categories anthropomorphism and animacy have some incosistencies,
# however since the questionnaire itself has been tested and used by many other people 
# in studies with more participants, we decided to keep all of the questions in those categories.

questions
psych::alpha(emp_mirror, check.keys=TRUE)
psych::alpha(emp_base, check.keys=TRUE)
psych::alpha(emp_mirror[,c(1,2,4,5,6)], check.keys=TRUE)
psych::alpha(emp_base[,c(1,2,4,5,6)], check.keys=TRUE)
#When the question consistency is checked per group, the 
#empathy questions seem to be borderline consistent.
# with the raw alpha score for the base condition alpha=0.73
# and the raw alpha score for the mirror condition alpha=0.69
#However, removing the third  question greatly improves the reliability
#of the mirror condition while only slighyly reducing the reliability of the base condition
# with the updates alpha= 0.7 and alpha=0.81 for the base and the mirror condition
# respectively
#Therefore this quetsion will be removed from both conditions.

questions

#after dropping that question, the updated graph looks like this:
averaged_data[,"empathy"] <- rowMeans(empathy[,c(1,2,4,5,6)])
averaged_data
emp_plot <- ggplot(averaged_data, aes(condition, empathy))+ geom_boxplot() + geom_point() + ylim(1,5)
emp_plot
## EMPATHY ###

# check the assumptions (normal distribution of the data, homogeneiy etc)

#normally distributed data. 
#plot the QQ plot:
mirror_data <- averaged_data[averaged_data$condition == "mirror",]
base_data <- averaged_data[averaged_data$condition == "base",]

qqplot.empathy <- qplot(sample = mirror_data$empathy, stat="qq")
qqplot.empathy
qqplot.empathy <- qplot(sample = base_data$empathy, stat="qq")
qqplot.empathy
# The data appears to be skewed.
hist.empathy <- ggplot(mirror_data, aes(empathy))+
  geom_histogram(aes(y = ..density..), colour = "black", fill = "white") + labs(x ="average empathy score mirror data", y = "Density")
hist.empathy + stat_function(fun = dnorm, args = list(mean = mean(mirror_data$empathy, na.rm =
                                                                 TRUE), sd = sd(mirror_data$empathy, na.rm = TRUE)), colour = "black", size = 1)
#If the shapiro is significant, the assumtion is violated
# Shapiro-Wilk test: unreliable for large datasets, which will not be a problem in our case
by(averaged_data$empathy, averaged_data$condition, shapiro.test)
shapiro.test(averaged_data$empathy)
#Normality conclusion: after visual inspection of the QQ plots and the histograms
# the data seems to deviate from the natural distribution. 
#moreover, the shapiro-wilk test also displayed a significant deviation from
# the natural distribution in the base condition with (W=0.72,p=0.023)

#Homogeneity of variance. If the levene test is significant, the assumption is violated:
leveneTest(averaged_data$empathy, averaged_data$condition)
#since the levene's test for homogeneity of variance is
# insignificant with F(1,6)=4, p= 0.0924, we conclude that this assumption is met

# assumption 3: interval must be the same
#This assumption is met since the steps on the Likert scale are spaced evenly
# We consider the differences between the 5 different values to be even

#assumption 3: independence:
# the data is assumed to be independent since each participant was assigned to only
# one condition and the participants did not communicate with each other about the 
# experiment or saw each other during the experiment.



# Since the normality assumption was violated, we will use the
#wilcox test instead of a parametric t-test
# to test our hypothesis that the mirroring condition will induce more empathy
# we will use an unpaired wilcox test and since our hypothesis was one-tailed
# with alt hyp = empathy of mirroring condition will be larger than in the base condition
# and null hyp = empathy in the mirroring condition is smaller or equal to the base condition
# we will use the paramter alternative = 
mirror_greater <-wilcox.test(averaged_data[averaged_data$condition =="base","empathy"],averaged_data[averaged_data$condition =="mirror","empathy"], paired= FALSE, alternative="less")
mirror_greater
#The null hypothesis as not rejected since W=14, p=0.9779 is statisticly
#insignificant with an alpha=0.05.
#this is unsurprising since the bar plots already showed us that
#the mirror condition mean is lower than the base condition mean.

#interestingly, if the alternative hypothesis was that the mirroring reduces
# empathy, there would be a significant effect with W = 14 and p =0.04427
mirror_smaller <-wilcox.test(averaged_data[averaged_data$condition =="base","empathy"],averaged_data[averaged_data$condition =="mirror","empathy"], paired= FALSE, alternative="greater")
mirror_smaller

# effect size 
z <- qnorm(mirror_greater$p.value)
r <- z / sqrt(nrow(averaged_data));r #effect size when testing the hypothesis:
# "mirroring codition will induse more empathy than the base condition" 0.7116328 (large effect)

mirror_smaller$p.value
z <- qnorm(mirror_smaller$p.value)
r <- z / sqrt(nrow(averaged_data));r # effect size when testing the hypothesis
# "mirroring condition induces less empathy than the base codition"  -0.6021508 (large effect)


by(averaged_data$empathy, averaged_data$condition, stat.desc)
# Mdn (median) of the base condition is Mdn=3.5
#The median of the mirror condition is Mdn=3.1
# the median of felt empathy for the mirroring condition was significantly 
# W = 14, p= 0.9779, r = -0.6021508 lower than the base condition
#the relationship between those two conditions can also be seen in the boxplot.

# use the godspeed questions to look for additional effects of our two conditions
# (the same t-test analysis as for empathy)

# find correlations between empathy scores and the 5 godspeed criteria.
# We want to discover the relationship between user perception of the robot and
# the amount of empathy they feel for the robot. 

#correlation analysis with godspeed and clarity of NAO and the setup


avg_empathy = sapply(averaged_data['empathy'],as.numeric)
avg_anthropomorphism = sapply(averaged_data['anthropomorphism'],as.numeric)
avg_animacy = sapply(averaged_data['animacy'],as.numeric)
avg_likeability = sapply(averaged_data['likeability'],as.numeric)
avg_intelligence = sapply(averaged_data['intelligence'],as.numeric)
avg_understanding = sapply(averaged_data['understanding'],as.numeric)


#p_cor_anthro_emp <- cor(avg_anthropomorphism, avg_empathy, method = c("pearson"));p_cor_anthro_emp
sp_cor_anthro_emp <- cor.test(avg_anthropomorphism, avg_empathy, method = c("spearman"));sp_cor_anthro_emp

#p_cor_animacy_emp <- cor(avg_animacy, avg_empathy, method = c("pearson"));p_cor_animacy_emp
sp_cor_animacy_emp <- cor.test(avg_animacy, avg_empathy, method = c("spearman"));sp_cor_animacy_emp

#p_cor_like_emp <- cor(avg_likeability, avg_empathy, method = c("pearson"));p_cor_like_emp
sp_cor_like_emp <- cor.test(avg_likeability, avg_empathy, method = c("spearman"));sp_cor_like_emp

#p_cor_int_emp <- cor(avg_intelligence, avg_empathy, method = c("pearson"));p_cor_int_emp
sp_cor_int_emp <- cor.test(avg_intelligence, avg_empathy, method = c("spearman"));sp_cor_int_emp

#p_cor_understanding_emp <- cor(avg_understanding, avg_empathy, method = c("pearson"));p_cor_understanding_emp
sp_cor_understanding_emp <- cor.test(avg_understanding, avg_empathy, method = c("spearman"));sp_cor_understanding_emp

