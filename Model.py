import parl
from parl import layers

# actor
class Actor(parl.Model):
    def __init__(self, act_dim, laser_num=7):
        self.laser_num = laser_num
        # --------------------------------------------
        # for baseline
        # self.fc1 = layers.fc(size=512, act="relu")
        # self.fc2 = layers.fc(size=512, act="relu")
        # self.fc3 = layers.fc(size=512, act="relu")
        # self.fc4 = layers.fc(size=act_dim, act="tanh") 
        # --------------------------------------------
        
        # --------------------------------------------
        # our method
        self.fc1 = layers.fc(size=32, act="relu")
        self.fc2 = layers.fc(size=64, act=None)
        self.fc3 = layers.fc(size=128, act=None)
        self.fc4 = layers.fc(size=32, act=None)
        self.fc5 = layers.fc(size=act_dim, act=None)
        self.res_fc = layers.fc(size=act_dim, act=None)
        # --------------------------------------------

    def policy(self, obs):
        out = self.fc1(obs)
        out = self.fc2(obs)
        out = self.fc3(out)
        out = self.fc4(out)
        out = self.fc5(out)

        return layers.tanh(out + self.res_fc(obs))

# critic
class Critic(parl.Model):
    def __init__(self):
        # --------------------------------------------
        # baseline
        # self.fc1 = layers.fc(size=512, act="relu")
        # self.fc2 = layers.fc(size=512, act="relu")
        # self.fc3 = layers.fc(size=512, act="relu")
        # self.fc4 = layers.fc(size=512, act="relu")
        # self.fc5 = layers.fc(size=1, act=None)
        # --------------------------------------------

        self.obs_fc1 = layers.fc(size=32, act="relu")
        self.obs_fc2 = layers.fc(size=64, act=None)
        self.obs_fc3 = layers.fc(size=128, act=None)

        self.act_fc1 = layers.fc(size=32, act="relu")
        self.act_fc2 = layers.fc(size=64, act="tanh")
        self.act_fc3 = layers.fc(size=128, act="tanh")

        self.total_fc1 = layers.fc(size=16, act="relu")
        self.total_fc2 = layers.fc(size=64, act=None)
        self.total_fc3 = layers.fc(size=128, act=None)

        self.re_fc1 = layers.fc(size=128 * 3, act="tanh")
        self.re_fc2 = layers.fc(size=256, act="tanh")
        self.re_fc3 = layers.fc(size=128, act="relu")
        self.re_fc4 = layers.fc(size=1, act="tanh")

    def value(self, obs, act):
        concat = layers.concat([obs, act], axis=1)
        # out = self.fc1(concat)
        # out = self.fc2(out)
        # out = self.fc3(out)
        # out = self.fc4(out)
        # out = self.fc5(out)
        o = self.obs_fc1(obs)
        o = self.obs_fc2(o)
        o = self.obs_fc3(o)

        a = self.act_fc1(act)
        a = self.act_fc2(a)
        a = self.act_fc3(a)

        c = self.total_fc1(concat)
        c = self.total_fc2(c)
        c = self.total_fc3(c)

        out = self.re_fc1(layers.concat([o, a, c], axis=1))
        out = self.re_fc2(out)
        out = self.re_fc3(out)
        out = self.re_fc4(out)

        return layers.squeeze(out, axes=[1])

# integate actor net and critic net together
class Model(parl.Model):
    def __init__(self, act_dim):
        self.actor_model = Actor(act_dim)
        self.critic_model = Critic()

    def policy(self, obs):
        return self.actor_model.policy(obs)

    def value(self, obs, act):
        return self.critic_model.value(obs, act)

    # get actor's parameter
    def get_actor_params(self):
        return self.actor_model.parameters()