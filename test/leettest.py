class TreeNode:
    def __init__(self, x):
        self.val = x
        self.left = None
        self.right = None


class Solution:
    def canCompleteCircuit(self, gas, cost):
        """
        :type gas: List[int]
        :type cost: List[int]
        :rtype: int
        """
        if len(gas) == 0 or len(cost) == 0 or sum(gas) < sum(cost):
            return -1
        position = 0
        balance = 0 # current tank balance
        for i in range(len(gas)):
            balance += gas[i] - cost[i] # update balance
            if balance < 0: # balance drops to negative, reset the start position
                balance = 0
                position = i+1
        return position

    def singleNumber(self, nums):
        ones, twos = 0, 0
        for i in nums:
            ones = (ones^i)& ~twos
            twos = (twos^i)& ~ones
        return ones
if __name__ == '__main__':
    gas = [2,3,4]
    cost = [3,4,3]
    nums = [1,2,1,2,1,2,3]
    myS = Solution()
    # print(myS.canCompleteCircuit(gas, cost))
    # print(myS.isPalindrome('aba', 0, -1))
    print(myS.singleNumber(nums))