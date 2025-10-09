// Copyright (C) 2024 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

export type ConversationRequest = {
  conversationId: string;
  userPrompt: Message;
};
export enum MessageRole {
  Assistant = "assistant",
  User = "user",
  System = "system",
}

export interface Message {
  role: MessageRole;
  content: string;
  time: number;
}

export interface Conversation {
  conversationId: string;
  title?: string;
  Messages: Message[];
}

export interface ConversationReducer {
  selectedConversationId: string;
  conversations: Conversation[];
  onGoingResults: { [conversationId: string]: string };
  modelName: string;
}
